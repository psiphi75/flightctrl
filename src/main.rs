#![allow(unused_imports)]
#![allow(clippy::single_component_path_imports)]

mod bmp280;

use std::fs;
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::path::PathBuf;
use std::sync::{Condvar, Mutex};
use std::{cell::RefCell, env, sync::atomic::*, sync::Arc, thread, time::*};

use anyhow::bail;
use embedded_svc::eth;
use embedded_svc::eth::{Eth, TransitionalState};
use embedded_svc::httpd::registry::*;
use embedded_svc::httpd::*;
use embedded_svc::httpd::*;
use embedded_svc::io;
use embedded_svc::ipv4;
use embedded_svc::mqtt::client::{Client, Connection, MessageImpl, Publish, QoS};
use embedded_svc::ping::Ping;
use embedded_svc::sys_time::SystemTime;
use embedded_svc::timer::TimerService;
use embedded_svc::timer::*;
use embedded_svc::wifi::*;
use embedded_svc::wifi::*;

use esp_idf_hal::delay;
use esp_idf_hal::i2c;
use esp_idf_hal::prelude::*;
use esp_idf_svc::eth::*;
use esp_idf_svc::eventloop::*;
use esp_idf_svc::httpd::ServerRegistry;
use esp_idf_svc::mqtt::client::*;
use esp_idf_svc::netif::*;
use esp_idf_svc::nvs::*;
use esp_idf_svc::ping;
use esp_idf_svc::sntp;
use esp_idf_svc::sysloop::*;
use esp_idf_svc::systime::EspSystemTime;
use esp_idf_svc::timer::*;
use esp_idf_svc::wifi::*;
use esp_idf_svc::wifi::*;

use log::info;
use mpu9250::vector::Vector;
// use bmp280;
use mpu9250::{calibration::Calibration, Mpu9250};
use mpu9250_i2c as mpu9250;

use esp_idf_sys::{self};

const SSID: &str = env!("RUST_ESP32_STD_DEMO_WIFI_SSID");
const PASS: &str = env!("RUST_ESP32_STD_DEMO_WIFI_PASS");

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
    esp_idf_svc::log::EspLogger::initialize_default();

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

    //
    // WiFi Initialisation
    //

    let netif_stack = Arc::new(EspNetifStack::new()?);
    let sys_loop_stack = Arc::new(EspSysLoopStack::new()?);
    let default_nvs = Arc::new(EspDefaultNvs::new()?);

    let mut wifi = wifi(
        netif_stack.clone(),
        sys_loop_stack.clone(),
        default_nvs.clone(),
    )?;

    //
    // Main Loop
    //

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

fn wifi(
    netif_stack: Arc<EspNetifStack>,
    sys_loop_stack: Arc<EspSysLoopStack>,
    default_nvs: Arc<EspDefaultNvs>,
) -> Result<Box<EspWifi>> {
    let mut wifi = Box::new(EspWifi::new(netif_stack, sys_loop_stack, default_nvs)?);

    info!("Wifi created, about to scan");

    let ap_infos = wifi.scan()?;

    let ours = ap_infos.into_iter().find(|a| a.ssid == SSID);

    let channel = if let Some(ours) = ours {
        info!(
            "Found configured access point {} on channel {}",
            SSID, ours.channel
        );
        Some(ours.channel)
    } else {
        info!(
            "Configured access point {} not found during scanning, will go with unknown channel",
            SSID
        );
        None
    };

    wifi.set_configuration(&Configuration::Mixed(
        ClientConfiguration {
            ssid: SSID.into(),
            password: PASS.into(),
            channel,
            ..Default::default()
        },
        AccessPointConfiguration {
            ssid: "aptest".into(),
            channel: channel.unwrap_or(1),
            ..Default::default()
        },
    ))?;

    info!("Wifi configuration set, about to get status");

    wifi.wait_status_with_timeout(Duration::from_secs(20), |status| !status.is_transitional())
        .map_err(|e| anyhow::anyhow!("Unexpected Wifi status: {:?}", e))?;

    let status = wifi.get_status();

    if let Status(
        ClientStatus::Started(ClientConnectionStatus::Connected(ClientIpStatus::Done(ip_settings))),
        ApStatus::Started(ApIpStatus::Done),
    ) = status
    {
        info!("Wifi connected");

        ping(&ip_settings)?;
    } else {
        bail!("Unexpected Wifi status: {:?}", status);
    }

    Ok(wifi)
}

fn ping(ip_settings: &ipv4::ClientSettings) -> Result<()> {
    info!("About to do some pings for {:?}", ip_settings);

    let ping_summary =
        ping::EspPing::default().ping(ip_settings.subnet.gateway, &Default::default())?;
    if ping_summary.transmitted != ping_summary.received {
        bail!(
            "Pinging gateway {} resulted in timeouts",
            ip_settings.subnet.gateway
        );
    }

    info!("Pinging done");

    Ok(())
}
