use std::thread::sleep;
use std::time::Duration;
use ublox7::{open_serial, read_ubx_response, send_ubx_command};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // In this example im not handling timeouts, you should make sure the device is ready before
    // attempting reading from serial, if you are getting timeout retry, serial is slow
    let mut port = open_serial("/dev/ttyACM0", 19200)?;

    // Poll NAV-POSLLH (Position, Longitude/Latitude)
    let class = 0x01;
    let id = 0x02;
    let payload: [u8; 0] = []; // set to 0

    send_ubx_command(&mut *port, class, id, &payload)?;
    println!("Command sent. Waiting for UBX response...");

    sleep(Duration::from_millis(300)); // Let device respond
    let response = read_ubx_response(&mut *port).ok_or("No UBX response received")?;

    if response.class == 0x01 && response.id == 0x02 && response.payload.len() >= 28 {
        let payload = &response.payload;
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

    Ok(())
}
