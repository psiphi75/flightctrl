// #![allow(unused_imports)]
#![allow(clippy::single_component_path_imports)]

mod bmp280;

use std::{cell::RefCell, thread, time::*};

use embedded_svc::httpd::*;
use esp_idf_hal::delay;
use esp_idf_hal::i2c;
use esp_idf_hal::prelude::*;

// use bmp280;
use mpu9250::{calibration::Calibration, Mpu9250};
use mpu9250_i2c as mpu9250;

use esp_idf_sys::{self};

#[allow(dead_code)]
const SSID: &str = "hp"; //env!("RUST_ESP32_STD_DEMO_WIFI_SSID");
#[allow(dead_code)]
const PASS: &str = "apollo11"; //env!("RUST_ESP32_STD_DEMO_WIFI_PASS");

const BMP280_ADDRESS: u8 = 0x76;

#[cfg(esp32s2)]
include!(env!("EMBUILD_GENERATED_SYMBOLS_FILE"));

#[cfg(esp32s2)]
const ULP: &[u8] = include_bytes!(env!("EMBUILD_GENERATED_BIN_FILE"));

thread_local! {
    static TLS: RefCell<u32> = RefCell::new(13);
}

fn main() -> Result<()> {
    esp_idf_sys::link_patches();

    // test_threads();

    let peripherals = Peripherals::take().unwrap();
    let i2c = peripherals.i2c0;
    let sda = peripherals.pins.gpio16;
    let scl = peripherals.pins.gpio17;

    let config = <i2c::config::MasterConfig as Default>::default().baudrate(400.kHz().into());
    let i2c0 =
        i2c::Master::<i2c::I2C0, _, _>::new(i2c, i2c::MasterPins { sda: sda, scl: scl }, config)?;

    // Set the calibration to the default setting.  This can
    // be set to a custom value specific for the device.
    let cal = Calibration {
        ..Default::default()
    };

    let mpu9250 = &mut Mpu9250::new(i2c0, delay::Ets, cal).unwrap();

    let i2c = peripherals.i2c1;
    let sda = peripherals.pins.gpio21;
    let scl = peripherals.pins.gpio22;
    let config = <i2c::config::MasterConfig as Default>::default().baudrate(400.kHz().into());
    let i2c1 =
        i2c::Master::<i2c::I2C1, _, _>::new(i2c, i2c::MasterPins { sda: sda, scl: scl }, config)?;

    // to create sensor with default configuration:
    let mut bmp = bmp280::BMP280::new(i2c1, BMP280_ADDRESS)?;
    println!("id={:?}", bmp.id());

    let control = bmp280::Control {
        osrs_t: bmp280::Oversampling::x16,
        osrs_p: bmp280::Oversampling::x16,
        mode: bmp280::PowerMode::Normal,
    };
    bmp.set_control(control);

    let config = bmp280::Config {
        t_sb: bmp280::Standby::ms0_5, // OFF
        filter: bmp280::Filter::off,
    };
    bmp.set_config(config);

    // Initialise with default settings
    mpu9250.init().unwrap();

    // Probe the temperature
    loop {
        let temp = mpu9250.get_temperature_celsius().unwrap();
        let mag = mpu9250.get_mag().unwrap();
        println!("Temp: {} °C, mag: {},{},{}", temp, mag.x, mag.y, mag.z);

        let pres = bmp.pressure();
        println!("{}\t{}", pres, bmp.temp());

        thread::sleep(Duration::from_millis(50));
    }
}

// fn test_threads() {
//     let mut children = vec![];

//     println!("Rust main thread: {:?}", thread::current());

//     TLS.with(|tls| {
//         println!("Main TLS before change: {}", *tls.borrow());
//     });

//     TLS.with(|tls| *tls.borrow_mut() = 42);

//     TLS.with(|tls| {
//         println!("Main TLS after change: {}", *tls.borrow());
//     });

//     for i in 0..5 {
//         // Spin up another thread
//         children.push(thread::spawn(move || {
//             println!("This is thread number {}, {:?}", i, thread::current());

//             TLS.with(|tls| *tls.borrow_mut() = i);

//             TLS.with(|tls| {
//                 println!("Inner TLS: {}", *tls.borrow());
//             });
//         }));
//     }

//     println!(
//         "About to join the threads. If ESP-IDF was patched successfully, joining will NOT crash"
//     );

//     for child in children {
//         // Wait for the thread to finish. Returns a result.
//         let _ = child.join();
//     }

//     TLS.with(|tls| {
//         println!("Main TLS after threads: {}", *tls.borrow());
//     });

//     thread::sleep(Duration::from_secs(2));

//     println!("Joins were successful.");
// }
