#![macro_use]
#![feature(type_alias_impl_trait)]

mod serial;

use {
    async_io::Async,
    embassy_time::{Duration, Timer},
    embedded_hal::digital::{ErrorType, OutputPin},
    embedded_io::{
        adapters::FromFutures,
        asynch::{Read, Write},
    },
    embedded_nal_async::TcpConnect,
    esp8266_at_driver::*,
    futures::io::BufReader,
    nix::sys::termios,
    serial::*,
    static_cell::StaticCell,
};

type SERIAL = FromFutures<BufReader<Async<SerialPort>>>;
type ENABLE = DummyPin;
type RESET = DummyPin;

// TODO: Modify to join your local network
const WIFI_SSID: &str = "my_ssid";
const WIFI_PSK: &str = "my_psk";

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    env_logger::builder()
        .filter_level(log::LevelFilter::Trace)
        .format_timestamp_nanos()
        .init();

    let baudrate = termios::BaudRate::B115200;
    let port = SerialPort::new("/dev/ttyUSB0", baudrate).unwrap();
    let port = Async::new(port).unwrap();
    let port = futures::io::BufReader::new(port);
    let port = FromFutures::new(port);

    let network = Esp8266Driver::new(port, DummyPin, DummyPin);
    static NETWORK: StaticCell<Esp8266Driver<SERIAL, ENABLE, RESET, 1>> = StaticCell::new();
    let network = NETWORK.init(network);

    spawner
        .spawn(net_task(network, WIFI_SSID.trim_end(), WIFI_PSK.trim_end()))
        .unwrap();

    loop {
        let mut connection = network
            .connect("192.168.1.2:8088".parse().unwrap())
            .await
            .unwrap();
        connection.write(b"ping").await.unwrap();
        let mut rx = [0; 4];
        let l = connection.read(&mut rx[..]).await.unwrap();
        assert_eq!(4, l);
        Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::task]
async fn net_task(
    driver: &'static Esp8266Driver<'static, SERIAL, ENABLE, RESET, 1>,
    ssid: &'static str,
    psk: &'static str,
) {
    loop {
        let _ = driver.run(ssid, psk).await;
    }
}

pub struct DummyPin;
impl ErrorType for DummyPin {
    type Error = ();
}

impl OutputPin for DummyPin {
    fn set_low(&mut self) -> Result<(), ()> {
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), ()> {
        Ok(())
    }
}
