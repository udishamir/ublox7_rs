# UBlox 7 UBX Protocol Rust Interface

This project provides a native Rust implementation to communicate with u-blox 7 GNSS modules using the UBX binary protocol via serial connection.
As part of my surrounding sensing project, i have decided to write drivers for some of the hardware im currently working with.

## UBX Protocol Overview

The UBX protocol is a binary communication protocol used by u-blox GPS modules. Unlike NMEA, which is human-readable, UBX is a compact and efficient binary format designed for precise control, polling, and high-frequency data streaming.

Manufacture: https://www.u-blox.com/en/about
spec: https://content.u-blox.com/sites/default/files/products/documents/u-blox7-V14_ReceiverDescriptionProtocolSpec_%28GPS.G7-SW-12001%29_Public.pdf

### UBX Protocol Subset Message Structure
```
Header     Class  ID     Length    Payload            Checksum
0xB5 0x62  1 byte 1 byte 2 bytes   variable (n bytes) 2 bytes (CK_A, CK_B)
```

### UBX Checksum Algorithm
The checksum is calculated over the `Class`, `ID`, `Length`, and `Payload` fields using an 8-bit Fletcher algorithm:
Fletcher algorithm is considered weak, it is efficient with low power communication / low power consumption devices.

Here I'm using 8 bit however Fletcher does support 32 and 64 bits version as well.
```rust
fn ubx_checksum(data: &[u8]) -> (u8, u8) {
    let mut ck_a: u8 = 0;
    let mut ck_b: u8 = 0;
    for byte in data {
        ck_a = ck_a.wrapping_add(*byte);
        ck_b = ck_b.wrapping_add(ck_a);
    }
    (ck_a, ck_b)
}
```

## Example: Polling Position (NAV-POSLLH), 
```rust
let class = 0x01;
let id = 0x02;
let payload = &[]; // Polling message, no payload
send_ubx_command(&mut port, class, id, payload)?;
```

### Parsing NAV-POSLLH Response
https://docs.ros.org/en/iron/p/ublox_ubx_msgs/interfaces/msg/UBXNavPosLLH.html


| Offset | Name   | Type | Description                     |
|--------|--------|------|---------------------------------|
| 0      | iTOW   | U4   | GPS time of week (ms)          |
| 4      | lon    | I4   | Longitude (1e-7 degrees)       |
| 8      | lat    | I4   | Latitude (1e-7 degrees)        |
| 12     | height | I4   | Height above ellipsoid (mm)   |
| 16     | hMSL   | I4   | Height above mean sea level   |
| 20     | hAcc   | U4   | Horizontal accuracy estimate   |
| 24     | vAcc   | U4   | Vertical accuracy estimate     |

Important: check your system system architecture for endinanness.

```rust
let lat = i32::from_le_bytes([payload[8], payload[9], payload[10], payload[11]]) as f64 * 1e-7;
let lon = i32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]) as f64 * 1e-7;
println!("Latitude: {:.7}, Longitude: {:.7}", lat, lon);
```

## Project Structure

- `lib.rs` + handles serial I/O, UBX protocol frame construction, checksum validation, and payload parsing.
- `main.rs` + CLI entry point for polling UBX data and printing results.

## License

This project is licensed under the MIT License.
**Disclaimer:** The author takes no responsibility for any issues, malfunctions, or damages that may arise from using this code. Use at your own risk.
