use core::future::Future;
use embedded_hal::spi::{ErrorKind, ErrorType, Operation};
use regiface::{Command, FromByteArray, ToByteArray};
use std::task::{Context, Poll, Waker};
use sx1262::{
    commands::{
        ClearDeviceErrors, GetDeviceErrors, GetDeviceErrorsResponse, GetIrqStatus,
        GetIrqStatusResponse, GetPacketStatusResponse, GetRawStatus, GetRssiInstResponse,
        GetRxBufferStatusResponse, GetStatsResponse, ImageCalibConfig, IrqMask, SetDio3AsTcxoCtrl,
        TcxoConfig, TcxoVoltage,
    },
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
        Err(Error::Deserialization)
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
        Err(Error::Deserialization)
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
