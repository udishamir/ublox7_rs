/*
    Ublox 7 Driver,

    By Udi Shamir,

    MIT License

    Copyright (c) 2025 Udi Shamir

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

use serialport::SerialPort;
use std::io;
use std::time::Duration;

#[derive(Debug)]
pub struct UbxMessage {
    pub class: u8,
    pub id: u8,
    pub payload: Vec<u8>,
}

pub struct Position {
    pub lat: f64,
    pub lon: f64,
    pub height_msl: f64,
    pub horizontal_accuracy: f64,
    pub vertical_accuracy: f64,
}

pub fn parse_nav_posllh(payload: &[u8]) -> Option<Position> {
    // Not valid message or not enough data received from the GPS sensor
    if payload.len() < 28 {
        return None;
    }

    let lon = i32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]) as f64 * 1e-7;
    let lat = i32::from_le_bytes([payload[8], payload[9], payload[10], payload[11]]) as f64 * 1e-7;
    let hmsl =
        i32::from_le_bytes([payload[16], payload[17], payload[18], payload[19]]) as f64 / 1000.0;
    let hacc =
        u32::from_le_bytes([payload[20], payload[21], payload[22], payload[23]]) as f64 / 1000.0;
    let vacc =
        u32::from_le_bytes([payload[24], payload[25], payload[26], payload[27]]) as f64 / 1000.0;

    Some(Position {
        lat,
        lon,
        height_msl: hmsl,
        horizontal_accuracy: hacc,
        vertical_accuracy: vacc,
    })
}

pub fn print_position(pos: &Position) {
    println!("\n===== UBX NAV-POSLLH Parsed Position =====");
    println!("Latitude:  {:.7} °", pos.lat);
    println!("Longitude: {:.7} °", pos.lon);
    println!("Altitude:  {:.2} m (MSL)", pos.height_msl);
    println!("H-Acc:     {:.2} m", pos.horizontal_accuracy);
    println!("V-Acc:     {:.2} m", pos.vertical_accuracy);
    println!("========================================\n");
}

pub fn open_serial(
    path: &str,
    baud_rate: u32,
) -> Result<Box<dyn SerialPort>, Box<dyn std::error::Error>> {
    let ports = serialport::available_ports()?;
    if !ports.iter().any(|p| p.port_name == path) {
        return Err(format!("Serial device not found: {}", path).into());
    }

    let port = serialport::new(path, baud_rate)
        .timeout(Duration::from_millis(200))
        .open()?;

    Ok(port)
}

/*
    This is implementation of the Fletcher-8,

    https://en.wikipedia.org/wiki/Fletcher%27s_checksum

    The purpose is preventing data corruption. Im using the 8 bit up to 255.
    Fletcher algorithm supports 8, 16, 32 and 64 bits.

    This algorithm is considered weak while it is good enough.
    It seems useful in low-power, low-bandwidth embedded systems or legacy protocols.
*/

fn ubx_checksum(data: &[u8]) -> (u8, u8) {
    let mut ck_a = 0u8;
    let mut ck_b = 0u8;
    for byte in data {
        ck_a = ck_a.wrapping_add(*byte);
        ck_b = ck_b.wrapping_add(ck_a);
    }
    (ck_a, ck_b)
}

// Ublox propietary protocol
pub fn send_ubx_command(
    port: &mut dyn SerialPort,
    class: u8,
    id: u8,
    payload: &[u8],
) -> io::Result<()> {
    let mut message: Vec<u8> = vec![
        0xB5,
        0x62, // UBX sync chars
        class,
        id,
        (payload.len() & 0xFF) as u8,
        ((payload.len() >> 8) & 0xFF) as u8,
    ];

    message.extend_from_slice(payload);

    let (ck_a, ck_b) = ubx_checksum(&message[2..]);
    message.push(ck_a);
    message.push(ck_b);

    port.write_all(&message)?;
    port.flush()?;
    Ok(())
}

pub fn read_ubx_response(port: &mut dyn SerialPort) -> Option<UbxMessage> {
    let mut buf = [0u8; 1024];
    match port.read(&mut buf) {
        Ok(n) => parse_ubx_message(&buf[..n]),
        Err(_) => None,
    }
}

fn parse_ubx_message(data: &[u8]) -> Option<UbxMessage> {
    if data.len() < 8 || data[0] != 0xB5 || data[1] != 0x62 {
        return None;
    }

    let class = data[2];
    let id = data[3];
    let len = u16::from_le_bytes([data[4], data[5]]) as usize;
    if data.len() < 8 + len {
        return None;
    }

    let payload = data[6..6 + len].to_vec();
    let ck_a = data[6 + len];
    let ck_b = data[7 + len];

    let (calc_a, calc_b) = ubx_checksum(&data[2..6 + len]);
    if ck_a == calc_a && ck_b == calc_b {
        Some(UbxMessage { class, id, payload })
    } else {
        None
    }
}
