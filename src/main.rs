// #![allow(unused_imports)]
#![allow(clippy::single_component_path_imports)]

mod bmp280;

use std::{cell::RefCell, thread, time::*};

use embedded_svc::httpd::*;
use embedded_svc::sys_time::SystemTime;
use esp_idf_hal::delay;
use esp_idf_hal::i2c;
use esp_idf_hal::prelude::*;

use esp_idf_svc::systime::EspSystemTime;
use mpu9250::vector::Vector;
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
    let i2c0 = i2c::Master::<i2c::I2C0, _, _>::new(i2c, i2c::MasterPins { sda, scl }, config)?;

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
    let i2c1 = i2c::Master::<i2c::I2C1, _, _>::new(i2c, i2c::MasterPins { sda, scl }, config)?;

    // to create sensor with default configuration:
    let mut bmp = bmp280::BMP280::new(i2c1, BMP280_ADDRESS)?;
    println!("id={:?}", bmp.id());

    // These BMP280 config and control settings ensure the most frequent
    // updates, but it requires us to filter it afterwards.
    //
    // The following values give an udpate rate of 125 Hz:
    //   config->t_sb    <== Standby::ms0_5
    //   config->filter  <== Filter::off
    //   control->osrs_t <== Oversampling::x1
    //   control->osrs_p <== Oversampling::x2
    //   control->mode   <== PowerMode::Normal
    //
    let config = bmp280::Config {
        t_sb: bmp280::Standby::ms0_5,
        filter: bmp280::Filter::off,
    };
    bmp.set_config(config);

    let control = bmp280::Control {
        osrs_t: bmp280::Oversampling::x1, // Temperature oversampling
        osrs_p: bmp280::Oversampling::x2, // Pressure oversampling
        mode: bmp280::PowerMode::Normal,
    };
    bmp.set_control(control);

    // Initialise with default settings
    mpu9250.init().unwrap();

    // bmp280 Timings
    const BMP280_UPDATE_MS: u32 = 1000 / 125;
    let mut bmp280_last_read_time = 0;

    // Accel and gyro timing
    const IMU_GA_UPDATE_MS: u32 = 1000 / 250;
    let mut imu_ga_last_read_time = 1; // Interleave with bmp280 reading

    // Accel and gyro timing
    const IMU_M_UPDATE_MS: u32 = 1000 / 100;
    let mut imu_m_last_read_time = 2;

    let mut pres = 0.0;
    let mut temp = 0.0;
    let mut accel = Vector {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    let mut gyro = Vector {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    let mut mag = Vector {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    let mut count = 0;

    loop {
        let now_ms = EspSystemTime {}.now().as_millis() as u32;

        let bmp280_do_read = (now_ms - bmp280_last_read_time) >= BMP280_UPDATE_MS;
        let imu_ga_do_read = (now_ms - imu_ga_last_read_time) >= IMU_GA_UPDATE_MS;
        let imu_m_do_read = (now_ms - imu_m_last_read_time) >= IMU_M_UPDATE_MS;
        let did_read = bmp280_do_read || imu_ga_do_read || imu_m_do_read;

        if bmp280_do_read {
            bmp280_last_read_time = now_ms;
            pres = match bmp.pressure() {
                Ok(r) => r,
                Err(e) => {
                    println!("Error reading pressure: {:?}", e);
                    pres
                }
            };
            temp = match bmp.temp() {
                Ok(r) => r,
                Err(e) => {
                    println!("Error reading temperature: {:?}", e);
                    temp
                }
            };
        }

        if imu_ga_do_read {
            imu_ga_last_read_time = now_ms;
            (accel, gyro) = match mpu9250.get_accel_gyro() {
                Ok(r) => r,
                Err(e) => {
                    println!("Error reading accel and gyro: {:?}", e);
                    (accel, gyro)
                }
            };
        }

        if imu_m_do_read {
            imu_m_last_read_time = now_ms;
            mag = match mpu9250.get_mag() {
                Ok(r) => r,
                Err(e) => {
                    println!("Error reading magnetometer: {:?}", e);
                    mag
                }
            };
        }

        count += 1;
        if imu_m_do_read && (count % 37 == 0) {
            print!(
                "{}: T: {:.1} C, pres: {:.2} hPa",
                now_ms,
                temp,
                pres / 100.0
            );
            print!(", accel: ({:.3},{:.3},{:.3})", accel.x, accel.y, accel.z);
            print!(", gryo: ({:.3},{:.3},{:.3})", gyro.x, gyro.y, gyro.z);
            println!(", mag: ({:.3},{:.3},{:.3})", mag.x, mag.y, mag.z);
        }

        if !did_read {
            thread::sleep(Duration::from_millis(1));
        }
    }
}
