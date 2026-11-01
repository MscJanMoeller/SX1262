# SX1262 Radio Driver

A type-safe embedded-hal driver for the Semtech SX1261/2 sub-GHz radio transceivers. This crate provides a robust interface for controlling these highly integrated, long range, low power radio transceivers designed for ISM band applications.

## Features

- **Frequency Range**: 150-960 MHz
- **Modulation Support**:
  - LoRa: SF5-12, BW 7.8-500kHz
  - (G)FSK: BR 0.6-300kbps
- **Output Power**:
  - SX1261: -17 to +15 dBm
  - SX1262: -9 to +22 dBm
- **High Sensitivity**: Down to -148 dBm
- **Power Management**: Integrated voltage regulation (DC-DC or LDO)
- **Flexible I/O**: Programmable DIO pins for interrupts and control
- **`no_std` Compatible**: Suitable for embedded systems
- **Type-safe Interface**: Built on `regiface` for reliable register access

## Usage

The main entry point is the `Device` type which wraps an SPI interface and provides methods for register access and command execution:

```rust
use sx1262::{Device, Error, SetStandby, StandbyConfig};
use embedded_hal::spi::SpiDevice;

fn configure_radio<SPI: SpiDevice>(spi: SPI) -> Result<Device<SPI>, Error<SPI::Error>> {
    let mut device = Device::new(spi);
    device.execute_command(SetStandby { config: StandbyConfig::Rc })?;
    Ok(device)
}
```

SPI errors are preserved; invalid command parameters and responses return
serialization or deserialization errors. Deserialization errors include the
opcode and first response byte, if any. The caller must handle BUSY timing,
IRQ waits, and the radio's required command sequence.

`TcxoConfig::delay` is a 24-bit count of 15.625 µs steps; larger values are
rejected. Status responses reject invalid operating modes and preserve
unrecognized command-status values and the raw byte. `GetRawStatus` is
available when the raw status byte is needed for diagnostics.

The driver is organized into modules for registers and commands:

- **device**: Main interface for hardware interaction
  - Wraps SPI communication
  - Provides register access methods
  - Handles command execution

- **registers**: Hardware register definitions
  - rf: Frequency, power, etc.
  - packet: Packet handling
  - dio: Digital I/O configuration
  - system: System configuration

- **commands**: Control interface
  - rf: RF/modulation configuration
  - dio: Interrupt control
  - operational: Mode control
  - status: Monitoring and statistics

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT license ([LICENSE-MIT](LICENSE-MIT))

at your option.
