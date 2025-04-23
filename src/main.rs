use serialport::SerialPort;
use ublox7::{open_serial, read_ubx_response, send_ubx_command};

const MAX_RETRY: u8 = 10;

fn parse_nav_svinfo(payload: &[u8]) {
    if payload.len() < 8 {
        println!("Payload too short for UBX-NAV-SVINFO");
        return;
    }

    let num_ch = payload[4] as usize;

    println!("\n===== UBX-NAV-SVINFO =====");
    println!("Number of channels: {}\n", num_ch);

    #[cfg(debug_assertions)]
    for i in 0..num_ch {
        let base = 8 + i * 12;
        if base + 12 > payload.len() {
            println!("Truncated satellite block");
            break;
        }

        let chn = payload[base];
        let svid = payload[base + 1];
        let flags = payload[base + 2];
        let quality = payload[base + 3];
        let cno = payload[base + 4]; // carrier-to-noise ratio
        let elev = payload[base + 5] as i8;
        let azim = i16::from_le_bytes([payload[base + 6], payload[base + 7]]);
        let pr_res = i32::from_le_bytes([
            payload[base + 8],
            payload[base + 9],
            payload[base + 10],
            payload[base + 11],
        ]);

        println!(
            "CH: {:2} | SVID: {:3} | C/N₀: {:2} dBHz | Elv: {:3}° | Azim: {:4}° | Quality: {} | Flags: 0x{:02X} | PR Res: {}",
            chn, svid, cno, elev, azim, quality, flags, pr_res
        );

        let svid = payload[base + 1];
        println!("Channel: {} {}", i, ublox7::svid_to_constellation(svid));
    }

    println!("======================================\n");
}

pub fn get_sat_info(port: &mut dyn SerialPort) -> Result<(), Box<dyn std::error::Error>> {
    use std::thread::sleep;
    use std::time::Duration;

    send_ubx_command(port, 0x01, 0x35, &[])?;
    println!("Command sent. Waiting for UBX-NAV-SAT response...");
    sleep(Duration::from_millis(500));

    if let Some(response) = read_ubx_response(port) {
        if response.class == 0x01 && response.id == 0x35 {
            println!("Raw NAV-SAT payload: {:?}", response.payload);
            if response.payload.len() < 8 {
                println!("Not enough data received for NAV-SAT header.");
                return Ok(());
            }

            let num_svs = response.payload[5] as usize;
            println!("Detected {} satellites:\n", num_svs);

            for i in 0..num_svs {
                let base = 8 + i * 12;
                if base + 12 > response.payload.len() {
                    println!("Truncated satellite block at index {}", i);
                    break;
                }

                let gnss_id = response.payload[base];
                let sv_id = response.payload[base + 1];
                let cno = response.payload[base + 2];
                let flags = response.payload[base + 3];
                let azimuth =
                    i16::from_le_bytes([response.payload[base + 4], response.payload[base + 5]]);
                let elevation = response.payload[base + 6] as i8;
                let orbit_source = response.payload[base + 7];

                println!(
                    "SV {:02}: GNSS={} | C/N₀={} dBHz | Az={}° | El={}° | Flags=0x{:02X} | OrbitSrc={}",
                    sv_id, gnss_id, cno, azimuth, elevation, flags, orbit_source
                );
            }
        } else {
            println!(
                "Unexpected UBX response: class=0x{:02X}, id=0x{:02X}, len={}",
                response.class,
                response.id,
                response.payload.len()
            );
        }
    } else {
        println!("❌ No UBX-NAV-SAT response received");
    }

    Ok(())
}

fn get_ublox7_message(
    mut port: Box<dyn SerialPort>,
    class: u8,
    id: u8,
    payload: [u8; 0],
) -> Result<(Box<dyn SerialPort>, ublox7::UbxMessage), Box<dyn SerialPort>> {
    for i in 1..=MAX_RETRY {
        if let Err(e) = send_ubx_command(&mut *port, class, id, &payload) {
            println!("Error: {}", e);
            continue;
        }
        if let Some(response) = read_ubx_response(&mut *port) {
            println!(
                "Got response from Ublox7, Attempt: {}/{} Class: {}, Id: {}",
                i, MAX_RETRY, response.class, response.id
            );
            // return
            if response.class == class && response.id == id {
                return Ok((port, response));
            }
        }
    }

    println!("Error: no response after: {} retries", MAX_RETRY);
    Err(port)
}

fn parse_ublox7_data(ubx_message: ublox7::UbxMessage) {
    if ubx_message.class == 0x01 && ubx_message.id == 0x02 && ubx_message.payload.len() >= 28 {
        let payload = &ubx_message.payload;
        let i_tow = u32::from_le_bytes(payload[0..4].try_into().unwrap());
        let lon = i32::from_le_bytes(payload[4..8].try_into().unwrap()) as f64 / 1e7;
        let lat = i32::from_le_bytes(payload[8..12].try_into().unwrap()) as f64 / 1e7;
        let height_ellipsoid =
            i32::from_le_bytes(payload[12..16].try_into().unwrap()) as f64 / 1000.0;
        let height_msl = i32::from_le_bytes(payload[16..20].try_into().unwrap()) as f64 / 1000.0;
        let h_acc = u32::from_le_bytes(payload[20..24].try_into().unwrap()) as f64 / 1000.0;
        let v_acc = u32::from_le_bytes(payload[24..28].try_into().unwrap()) as f64 / 1000.0;

        println!("NAV-POSLLH:");
        println!("  Time of Week: {} ms", i_tow);
        println!("  Longitude: {:.7}°", lon);
        println!("  Latitude: {:.7}°", lat);
        println!("  Height (ellipsoid): {:.3} m", height_ellipsoid);
        println!("  Height (MSL): {:.3} m", height_msl);
        println!("  Horizontal Accuracy: {:.3} m", h_acc);
        println!("  Vertical Accuracy: {:.3} m", v_acc);
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // In this example im not handling timeouts, you should make sure the device is ready before
    // attempting reading from serial, if you are getting timeout retry, serial is slow
    let mut port = open_serial("/dev/ttyACM0", 19200)?;

    // Attempting to retrieve vehichle (satelitte) information such as: {gps, glonass, beiduo}
    if let Some(sat_response_svinfo) = read_ubx_response(&mut *port) {
        if sat_response_svinfo.class == 0x01 && sat_response_svinfo.id == 0x30 {
            parse_nav_svinfo(&sat_response_svinfo.payload);
        }
    }

    let class = 0x01;
    let id = 0x02;
    let payload: [u8; 0] = []; // set to 0

    println!("Command sent. Waiting for UBX response...");

    let ublox7_response = get_ublox7_message(port, class, id, payload);
    match ublox7_response {
        Ok(ubx_message) => {
            parse_ublox7_data(ubx_message.1);
        }
        Err(e) => {
            println!("Error, failed communicating with Ublox7: {:?}", e);
        }
    }

    Ok(())
}
