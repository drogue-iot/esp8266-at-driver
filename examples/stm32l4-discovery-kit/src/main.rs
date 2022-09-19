#![no_std]
#![no_main]
#![macro_use]
#![allow(incomplete_features)]
#![allow(unused_imports)]
#![allow(dead_code)]
#![feature(generic_associated_types)]
#![feature(type_alias_impl_trait)]
#![feature(concat_idents)]

use embassy_stm32::{
    exti::ExtiInput,
    flash::Flash,
    time::hz,
    gpio::{Input, Level, Output, Speed, Pull},
    peripherals::{DMA2_CH1, DMA2_CH2, PB13, PE0, PE1, PE8, SPI3},
    spi,
};
use embassy_time::Duration;
use embedded_nal_async::{AddrType, Dns, IpAddr, Ipv4Addr, SocketAddr, TcpConnect};
use embedded_io::asynch::{Write, Read};
use es_wifi_driver::*;
use static_cell::StaticCell;

use defmt_rtt as _;
use panic_probe as _;

// Makes it simpler to use the type:
type SPI = spi::Spi<'static, SPI3, DMA2_CH2, DMA2_CH1>;
type SpiError = spi::Error;
pub type WifiWake = Output<'static, PB13>;
pub type WifiReset = Output<'static, PE8>;
pub type WifiCs = Output<'static, PE0>;
pub type WifiReady = ExtiInput<'static, PE1>;
pub type WifiAdapter = EsWifi<SPI, WifiCs, WifiReset, WifiWake, WifiReady>;

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    let p = embassy_stm32::init(Default::default());
    let spi = spi::Spi::new(
        p.SPI3,
        p.PC10,
        p.PC12,
        p.PC11,
        p.DMA2_CH2,
        p.DMA2_CH1,
        hz(4_000_000),
        spi::Config::default(),
    );

    let _boot = Output::new(p.PB12, Level::Low, Speed::VeryHigh);
    let wake = Output::new(p.PB13, Level::Low, Speed::VeryHigh);
    let reset = Output::new(p.PE8, Level::Low, Speed::VeryHigh);
    let cs = Output::new(p.PE0, Level::High, Speed::VeryHigh);
    let ready = Input::new(p.PE1, Pull::Up);
    let ready = ExtiInput::new(ready, p.EXTI1);

    let adapter = WifiAdapter::new(spi, cs, reset, wake, ready);
    static NETWORK: StaticCell<WifiAdapter> = StaticCell::new();
    let network = NETWORK.init(adapter);

    spawner.spawn(network_task(network)).unwrap();

    loop {
        let mut connection = network.connect("127.0.0.1:8080".parse().unwrap()).await.unwrap();
        connection.write(b"ping").await.unwrap();
        let mut rx = [0; 4];
        let l = connection.read(&mut rx[..]).await.unwrap();
        assert_eq!(4, l);
    }
}

#[embassy_executor::task]
async fn network_task(adapter: &'static WifiAdapter) {
    loop {
        // TODO: Edit these to match your home network
        let _ = adapter.run("foo", "1234").await;
    }
}
