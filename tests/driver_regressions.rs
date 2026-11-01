use core::future::Future;
use embedded_hal::spi::{ErrorKind, ErrorType, Operation};
use regiface::{Command, FromByteArray, ToByteArray};
use std::task::{Context, Poll, Waker};
use sx1262::{
    commands::{
        AddressFiltering, BufferBaseAddressConfig, Calibrate, CalibrationConfig, ClearDeviceErrors,
        CommandStatusError, CrcType, DioIrqConfig, GFSKPacketHeaderType, GFSKPacketParams,
        GetDeviceErrors, GetDeviceErrorsResponse, GetIrqStatus, GetIrqStatusResponse,
        GetPacketStatusResponse, GetRawStatus, GetRssiInstResponse, GetRxBufferStatusResponse,
        GetStatsResponse, GfskBandwidth, GfskModParams, GfskPulseShape, ImageCalibConfig, IrqMask,
        ModulationParams, OperatingMode, PacketParams, PacketType, PreambleDetectorLength,
        RegulatorMode, RfFrequencyConfig, RxDutyCycleConfig, RxMode, SetBufferBaseAddress,
        SetDio3AsTcxoCtrl, SetDioIrqParams, SetModulationParams, SetPacketParams, SetPacketType,
        SetRegulatorMode, SetRfFrequency, SetRx, SetRxDutyCycle, SetStandby, StandbyConfig,
        TcxoConfig, TcxoVoltage,
    },
    registers::SyncWord,
    Device, Error,
};

fn ready<F: Future>(future: F) -> F::Output {
    let mut future = std::pin::pin!(future);
    match future
        .as_mut()
        .poll(&mut Context::from_waker(Waker::noop()))
    {
        Poll::Ready(value) => value,
        Poll::Pending => panic!("mock operation unexpectedly pending"),
    }
}

#[derive(Default)]
struct MockSpi {
    writes: Vec<u8>,
    response: Vec<u8>,
    fail: bool,
}

impl ErrorType for MockSpi {
    type Error = ErrorKind;
}

impl MockSpi {
    fn transact(&mut self, ops: &mut [Operation<'_, u8>]) -> Result<(), ErrorKind> {
        if self.fail {
            return Err(ErrorKind::ModeFault);
        }
        for op in ops {
            match op {
                Operation::Write(bytes) => self.writes.extend_from_slice(bytes),
                Operation::Read(bytes) => {
                    assert_eq!(bytes.len(), self.response.len());
                    bytes.copy_from_slice(&self.response);
                }
                _ => panic!("unexpected SPI operation"),
            }
        }
        Ok(())
    }
}

impl embedded_hal::spi::SpiDevice for MockSpi {
    fn transaction(&mut self, ops: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        self.transact(ops)
    }
}

impl embedded_hal_async::spi::SpiDevice for MockSpi {
    async fn transaction(&mut self, ops: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        self.transact(ops)
    }
}

#[test]
fn irq_bits_and_unknown_flags_are_preserved() {
    assert_eq!(IrqMask::HEADER_VALID.bits(), 0x0010);
    assert_eq!(IrqMask::HEADER_ERROR.bits(), 0x0020);
    assert_eq!(IrqMask::CRC_ERROR.bits(), 0x0040);
    assert_eq!(IrqMask::CAD_DONE.bits(), 0x0080);
    assert_eq!(IrqMask::CAD_DETECTED.bits(), 0x0100);
    assert_eq!(IrqMask::TIMEOUT.bits(), 0x0200);
    let response = GetIrqStatusResponse::from_bytes([0x24, 0x82, 0x00]).unwrap();
    assert_eq!(response.irq_mask.bits(), 0x8200);
}

#[test]
fn tcxo_encoding_checks_its_24_bit_limit() {
    let params = |delay| TcxoConfig {
        voltage: TcxoVoltage::V3_0,
        delay,
    };
    assert_eq!(params(320).to_bytes().unwrap(), [0x06, 0x00, 0x01, 0x40]);
    assert_eq!(
        params(0xff_ffff).to_bytes().unwrap(),
        [0x06, 0xff, 0xff, 0xff]
    );
    assert!(params(0x100_0000).to_bytes().is_err());
    let mut device = Device::new(MockSpi::default());
    device
        .execute_command(SetDio3AsTcxoCtrl {
            config: params(320),
        })
        .unwrap();
    assert_eq!(device.release().writes, [0x97, 0x06, 0, 1, 0x40]);
}

#[test]
fn spi_accessors_expose_the_wrapped_device_without_releasing_it() {
    let mut device = Device::new(MockSpi::default());
    assert!(device.spi().writes.is_empty());
    device
        .execute_command(SetStandby {
            config: StandbyConfig::Rc,
        })
        .unwrap();
    assert_eq!(device.spi().writes, [0x80, 0x00]);
    device.spi_mut().writes.clear();
    assert!(device.spi().writes.is_empty());
    assert!(device.release().writes.is_empty());
}

#[test]
fn raw_status_allows_diagnostics_for_any_byte() {
    for status in [0, 0x24, 0xff] {
        let mut device = Device::new(MockSpi {
            response: vec![status],
            ..MockSpi::default()
        });
        assert_eq!(
            ready(device.execute_command_async(GetRawStatus)).unwrap(),
            status
        );
        assert_eq!(device.release().writes, [0xc0]);
    }
}

#[test]
fn invalid_statuses_are_errors_instead_of_panics() {
    assert!(GetIrqStatusResponse::from_bytes([0, 0, 0]).is_err());
    assert!(GetRssiInstResponse::from_bytes([0, 0]).is_err());
    assert!(GetRxBufferStatusResponse::from_bytes([0, 0, 0]).is_err());
    assert!(GetPacketStatusResponse::from_bytes([0, 0, 0, 0]).is_err());
    assert!(GetDeviceErrorsResponse::from_bytes([0, 0, 0]).is_err());
    assert!(GetStatsResponse::from_bytes([0, 0, 0, 0, 0, 0, 0]).is_err());

    let errors = GetDeviceErrorsResponse::from_bytes([0x24, 0x80, 0x20]).unwrap();
    assert_eq!(errors.errors.raw, 0x8020);
    assert!(errors.errors.xosc_start_err);
}

#[test]
fn preserves_unrecognized_command_status_from_radio() {
    let response = GetDeviceErrorsResponse::from_bytes([0xa2, 0, 0x20]).unwrap();
    assert_eq!(response.status.raw, 0xa2);
    assert_eq!(response.status.mode, OperatingMode::StandbyRc);
    assert_eq!(
        response.status.cmd_status,
        Err(CommandStatusError::InvalidValue(1))
    );
    assert_eq!(response.errors.raw, 0x20);
}

#[test]
fn image_calibration_matches_datasheet_bands() {
    let bands = [
        (430, 440, ImageCalibConfig::BAND_430_440_MHZ, [0x6b, 0x6f]),
        (470, 510, ImageCalibConfig::BAND_470_510_MHZ, [0x75, 0x81]),
        (779, 787, ImageCalibConfig::BAND_779_787_MHZ, [0xc1, 0xc5]),
        (863, 870, ImageCalibConfig::BAND_863_870_MHZ, [0xd7, 0xdb]),
        (902, 928, ImageCalibConfig::BAND_902_928_MHZ, [0xe1, 0xe9]),
    ];
    for (min_mhz, max_mhz, band, bytes) in bands {
        let (min_hz, max_hz) = (min_mhz * 1_000_000, max_mhz * 1_000_000);
        assert_eq!(band.to_bytes().unwrap(), bytes);
        assert_eq!(ImageCalibConfig::covering(min_hz, max_hz), band);
        assert_eq!(ImageCalibConfig::for_frequency_hz(min_hz), band);
        assert_eq!(ImageCalibConfig::for_frequency_hz(max_hz), band);
        assert_eq!(
            ImageCalibConfig::for_frequency_hz((min_hz + max_hz) / 2),
            band
        );
    }
}

#[test]
fn image_calibration_outside_bands_covers_the_frequency() {
    let expected = [
        (150_000_000, [35, 39]),
        (315_000_000, [77, 81]),
        (870_350_000, [0xd7, 0xdb]),
        (928_000_001, [229, 235]),
        (960_000_000, [237, 243]),
    ];
    for (frequency_hz, codes) in expected {
        let config = ImageCalibConfig::for_frequency_hz(frequency_hz);
        assert_eq!(config.to_bytes().unwrap(), codes, "{frequency_hz} Hz");
        assert!(config.freq1 % 2 == 1 && config.freq2 % 2 == 1);
        assert!(u32::from(config.freq1) * 4_000_000 < frequency_hz);
        assert!(u32::from(config.freq2) * 4_000_000 > frequency_hz);
    }
    assert_eq!(
        ImageCalibConfig::covering(0, u32::MAX).to_bytes().unwrap(),
        [0, 255]
    );
}

#[test]
fn clear_errors_and_calibration_have_datasheet_bytes() {
    assert_eq!(ClearDeviceErrors::id(), 0x07);
    assert_eq!(
        ClearDeviceErrors.invoking_parameters().to_bytes().unwrap(),
        [0, 0]
    );
    assert_eq!(
        ImageCalibConfig::BAND_863_870_MHZ.to_bytes().unwrap(),
        [0xd7, 0xdb]
    );
    let mut device = Device::new(MockSpi::default());
    device.execute_command(ClearDeviceErrors).unwrap();
    assert_eq!(device.release().writes, [0x07, 0, 0]);
}

#[test]
fn synchronous_device_reports_spi_serialization_and_deserialization_errors() {
    let mut device = Device::new(MockSpi {
        fail: true,
        ..MockSpi::default()
    });
    assert!(matches!(
        device.execute_command(GetRawStatus),
        Err(Error::Spi(ErrorKind::ModeFault))
    ));

    let mut device = Device::new(MockSpi::default());
    assert!(matches!(
        device.execute_command(SetDio3AsTcxoCtrl {
            config: TcxoConfig {
                voltage: TcxoVoltage::V3_0,
                delay: 0x100_0000,
            },
        }),
        Err(Error::Serialization)
    ));
    assert!(device.release().writes.is_empty());

    let mut device = Device::new(MockSpi {
        response: vec![0, 0, 0],
        ..MockSpi::default()
    });
    assert!(matches!(
        device.execute_command(GetDeviceErrors),
        Err(Error::Deserialization {
            opcode: 0x17,
            first_byte: Some(0),
        })
    ));
}

#[test]
fn async_device_preserves_transport_and_parser_errors() {
    let mut device = Device::new(MockSpi {
        fail: true,
        ..MockSpi::default()
    });
    assert!(matches!(
        ready(device.execute_command_async(GetIrqStatus)),
        Err(Error::Spi(ErrorKind::ModeFault))
    ));

    let mut device = Device::new(MockSpi {
        response: vec![0, 0, 0],
        ..MockSpi::default()
    });
    assert!(matches!(
        ready(device.execute_command_async(GetIrqStatus)),
        Err(Error::Deserialization {
            opcode: 0x12,
            first_byte: Some(0),
        })
    ));

    let mut device = Device::new(MockSpi::default());
    assert!(matches!(
        ready(device.execute_command_async(SetDio3AsTcxoCtrl {
            config: TcxoConfig {
                voltage: TcxoVoltage::V3_0,
                delay: 0x100_0000,
            },
        })),
        Err(Error::Serialization)
    ));
    assert!(device.release().writes.is_empty());
}

fn wire<C: Command<IdType = u8>>(command: C) -> Vec<u8> {
    let mut device = Device::new(MockSpi::default());
    assert!(device.execute_command(command).is_ok());
    device.release().writes
}

fn rejected<C: Command<IdType = u8>>(command: C) {
    let mut device = Device::new(MockSpi::default());
    assert!(matches!(
        device.execute_command(command),
        Err(Error::Serialization)
    ));
    assert!(device.release().writes.is_empty());
}

fn gfsk(bit_rate: u32, freq_deviation: u32) -> SetModulationParams {
    SetModulationParams {
        params: ModulationParams::Gfsk(GfskModParams {
            bit_rate,
            pulse_shape: GfskPulseShape::NoFilter,
            bandwidth: GfskBandwidth::Bw1173,
            freq_deviation,
        }),
    }
}

#[test]
fn gfsk_receive_commands_have_datasheet_bytes() {
    assert_eq!(
        wire(SetPacketType {
            packet_type: PacketType::Gfsk
        }),
        [0x8a, 0]
    );
    assert_eq!(
        wire(SetRfFrequency {
            config: RfFrequencyConfig {
                frequency: 868_350_000
            }
        }),
        [0x86, 0x36, 0x45, 0x99, 0x99]
    );
    assert_eq!(
        wire(gfsk(17_200, 35_000)),
        [0x8b, 0x00, 0xe8, 0x8e, 0x00, 0x0b, 0x00, 0x8f, 0x5c]
    );
    assert_eq!(
        wire(SetPacketParams {
            params: PacketParams::GFSK(GFSKPacketParams {
                preamble_length: 48,
                preamble_detector_length: PreambleDetectorLength::Bits16,
                sync_word_length: 16,
                address_filtering: AddressFiltering::Disable,
                packet_type: GFSKPacketHeaderType::Fixed,
                payload_length: 64,
                crc_type: CrcType::CrcOff,
                whitening_enable: false,
            })
        }),
        [0x8c, 0, 48, 0x05, 16, 0, 0, 64, 1, 0]
    );
    let mut device = Device::new(MockSpi::default());
    device
        .write_register(SyncWord {
            value: [0x2d, 0xd4, 0, 0, 0, 0, 0, 0],
        })
        .unwrap();
    assert_eq!(
        device.release().writes,
        [0x0d, 0x06, 0xc0, 0x2d, 0xd4, 0, 0, 0, 0, 0, 0]
    );
    assert_eq!(
        wire(SetBufferBaseAddress {
            config: BufferBaseAddressConfig {
                tx_base_addr: 0,
                rx_base_addr: 0
            }
        }),
        [0x8f, 0, 0]
    );
    assert_eq!(
        wire(SetDioIrqParams {
            config: DioIrqConfig {
                irq_mask: IrqMask::RX_DONE
                    | IrqMask::PREAMBLE_DETECTED
                    | IrqMask::SYNC_WORD_VALID
                    | IrqMask::HEADER_ERROR
                    | IrqMask::CRC_ERROR
                    | IrqMask::TIMEOUT,
                dio1_mask: IrqMask::RX_DONE
                    | IrqMask::HEADER_ERROR
                    | IrqMask::CRC_ERROR
                    | IrqMask::TIMEOUT,
                dio2_mask: IrqMask::empty(),
                dio3_mask: IrqMask::empty(),
            }
        }),
        [0x08, 0x02, 0x6e, 0x02, 0x62, 0, 0, 0, 0]
    );
    assert_eq!(
        wire(SetRegulatorMode {
            mode: RegulatorMode::LdoOnly
        }),
        [0x96, 0]
    );
    assert_eq!(
        wire(Calibrate {
            config: CalibrationConfig::all()
        }),
        [0x89, 0x7f]
    );
    assert_eq!(
        wire(SetRx {
            mode: RxMode::Continuous
        }),
        [0x82, 0xff, 0xff, 0xff]
    );
    assert_eq!(
        wire(SetRx {
            mode: RxMode::Single
        }),
        [0x82, 0, 0, 0]
    );
}

#[test]
fn gfsk_rssi_is_reported_in_half_dbm() {
    let response = GetPacketStatusResponse::from_bytes([0x24, 0x02, 0x70, 0x67]).unwrap();
    assert_eq!(response.packet_status.gfsk_rssi_avg_half_dbm(), -103);
}

#[test]
fn gfsk_bandwidth_settings_match_datasheet_table() {
    let table = [
        (4_800, 0x1f),
        (5_800, 0x17),
        (7_300, 0x0f),
        (9_700, 0x1e),
        (11_700, 0x16),
        (14_600, 0x0e),
        (19_500, 0x1d),
        (23_400, 0x15),
        (29_300, 0x0d),
        (39_000, 0x1c),
        (46_900, 0x14),
        (58_600, 0x0c),
        (78_200, 0x1b),
        (93_800, 0x13),
        (117_300, 0x0b),
        (156_200, 0x1a),
        (187_200, 0x12),
        (232_300, 0x0a),
        (312_000, 0x19),
        (373_600, 0x11),
        (467_000, 0x09),
    ];
    for (hz, code) in table {
        let bandwidth = GfskBandwidth::from_hz(hz).unwrap();
        assert_eq!(bandwidth as u8, code);
        assert_eq!(bandwidth.hz(), hz);
    }
    assert_eq!(GfskBandwidth::from_hz(125_000), None);
    assert_eq!(GfskBandwidth::from_hz(0), None);
}

#[test]
fn gfsk_modulation_rejects_values_outside_radio_limits() {
    assert_eq!(
        wire(gfsk(600, 600)),
        [0x8b, 0x1a, 0x0a, 0xaa, 0x00, 0x0b, 0x00, 0x02, 0x75]
    );
    assert_eq!(
        wire(gfsk(300_000, 200_000)),
        [0x8b, 0x00, 0x0d, 0x55, 0x00, 0x0b, 0x03, 0x33, 0x33]
    );
    for (bit_rate, deviation) in [
        (0, 35_000),
        (599, 35_000),
        (300_001, 35_000),
        (17_200, 0),
        (17_200, 599),
        (17_200, 200_001),
    ] {
        rejected(gfsk(bit_rate, deviation));
    }
    assert!(ModulationParams::Gfsk(GfskModParams {
        bit_rate: 0,
        pulse_shape: GfskPulseShape::NoFilter,
        bandwidth: GfskBandwidth::Bw1173,
        freq_deviation: 35_000,
    })
    .to_bytes()
    .is_err());
}

#[test]
fn rf_frequency_rejects_values_outside_radio_range() {
    let frequency = |frequency| SetRfFrequency {
        config: RfFrequencyConfig { frequency },
    };
    assert_eq!(wire(frequency(150_000_000)), [0x86, 0x09, 0x60, 0x00, 0x00]);
    assert_eq!(wire(frequency(960_000_000)), [0x86, 0x3c, 0x00, 0x00, 0x00]);
    rejected(frequency(149_999_999));
    rejected(frequency(960_000_001));
}

#[test]
fn receive_timers_use_24_bit_fields() {
    assert_eq!(
        wire(SetRx {
            mode: RxMode::Timed(0xff_fffe)
        }),
        [0x82, 0xff, 0xff, 0xfe]
    );
    rejected(SetRx {
        mode: RxMode::Timed(0x100_0000),
    });
    assert_eq!(
        wire(SetRxDutyCycle {
            config: RxDutyCycleConfig {
                rx_period: 0x01_0203,
                sleep_period: 0x04_0506,
            }
        }),
        [0x94, 1, 2, 3, 4, 5, 6]
    );
    for (rx_period, sleep_period) in [(0x100_0000, 0), (0, 0x100_0000)] {
        rejected(SetRxDutyCycle {
            config: RxDutyCycleConfig {
                rx_period,
                sleep_period,
            },
        });
    }
}
