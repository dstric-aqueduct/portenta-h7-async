#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::mem::MaybeUninit;
use core::ptr::addr_of_mut;

use defmt::*;
use embassy_executor::{main, task, Spawner};
use embassy_net::tcp::TcpSocket;
use embassy_net::{Ipv4Address, Ipv4Cidr, Stack, StackResources};
use embassy_stm32::eth::generic_smi::GenericSMI;
use embassy_stm32::eth::{Ethernet, PacketQueue};
use embassy_stm32::peripherals::ETH;
use embassy_stm32::rng::Rng;
use embassy_stm32::{bind_interrupts, eth, peripherals, rng, Config};
use embassy_time::Timer;
use embedded_io_async::Write;
use heapless::Vec;
use rand_core::RngCore;
use static_cell::make_static;
use {defmt_rtt as _, panic_probe as _};

use embassy_stm32::{
    gpio::{Level, Output, Speed},
    usb_otg::{Driver, Instance},
};

bind_interrupts!(struct Irqs {
    ETH => eth::InterruptHandler;
    RNG => rng::InterruptHandler<peripherals::RNG>;
});

type Device = Ethernet<'static, ETH, GenericSMI>;

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<Device>) -> ! {
    stack.run().await
}

// Then we can use it!
static mut rx_buffer: [u8; 1024] = [0_u8; 1024];
static mut tx_buffer: [u8; 1024] = [0_u8; 1024];

// This data will be held by Net through a mutable reference
pub struct NetStorageStatic {
    socket_storage: StackResources::<3>,
}

// MaybeUninit allows us write code that is correct even if STORE is not
// initialised by the runtime
static mut STORE: MaybeUninit<NetStorageStatic> = MaybeUninit::uninit();

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    portenta_h7_async::sys::Clk::new()
        .reset()
        .enable_ext_clock();

    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = Some(HSIPrescaler::DIV1);
        config.rcc.csi = true;
        config.rcc.hsi48 = Some(Default::default()); // needed for RNG
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL50,
            divp: Some(PllDiv::DIV2),
            divq: None,
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P; // 400 Mhz
        config.rcc.ahb_pre = AHBPrescaler::DIV2; // 200 Mhz
        config.rcc.apb1_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb2_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb3_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb4_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.voltage_scale = VoltageScale::Scale1;
    }
    let p = embassy_stm32::init(config);

    // // User leds
    // let led_red = Output::new(p.PK5, Level::High, Speed::Low);
    // let led_green = Output::new(p.PK6, Level::High, Speed::Low);
    let led_blue = Output::new(p.PK7, Level::High, Speed::Low);

    // info!("Hello World!");

    // Generate random seed.
    let mut rng = Rng::new(p.RNG, Irqs);
    let mut seed = [0; 8];
    rng.fill_bytes(&mut seed);
    let seed = u64::from_le_bytes(seed);

    let mac_addr = [0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];

    let device = Ethernet::new(
        make_static!(PacketQueue::<16, 16>::new()),
        p.ETH,
        Irqs,
        p.PA1,
        p.PA2,
        p.PC1,
        p.PA7,
        p.PC4,
        p.PC5,
        p.PG13,
        p.PG12,
        p.PG11,
        GenericSMI::new(0),
        mac_addr,
    );

    // let config = embassy_net::Config::dhcpv4(Default::default());
    let config = embassy_net::Config::ipv4_static(embassy_net::StaticConfigV4 {
        address: Ipv4Cidr::new(Ipv4Address::new(169, 254, 122, 147), 24),
        dns_servers: Vec::new(),
        gateway: Some(Ipv4Address::new(169, 254, 255, 255)),
    });

     // unsafe: mutable reference to static storage, we only do this once
     let store = unsafe {
        let store_ptr = STORE.as_mut_ptr();

        // Initialise the socket_storage field. Using `write` instead of
        // assignment via `=` to not call `drop` on the old, uninitialised
        // value
        let t = StackResources::<3>::new();
        addr_of_mut!((*store_ptr).socket_storage).write(t);

        // Now that all fields are initialised we can safely use
        // assume_init_mut to return a mutable reference to STORE
        STORE.assume_init_mut()
    };

    // Init network stack
    let stack = &*make_static!(Stack::new(
        device,
        config,
        &mut store.socket_storage,
        seed
    ));

    spawner.spawn(blink_led_blue(led_blue)).unwrap();

    // Launch network task
    unwrap!(spawner.spawn(net_task(&stack)));

    // Ensure DHCP configuration is up before trying connect
    stack.wait_config_up().await;

    info!("Network task initialized");

    loop {
        let mut socket = unsafe { TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer) };

        socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

        // You need to start a server on the host machine, for example: `nc -l 8000`
        let remote_endpoint = (Ipv4Address::new(169, 254, 122, 148), 8000);
        info!("connecting...");
        let r = socket.connect(remote_endpoint).await;
        if let Err(e) = r {
            info!("connect error: {:?}", e);
            Timer::after_secs(1).await;
            continue;
        }
        info!("connected!");
        loop {
            let r = socket.write_all(b"Hello\n").await;
            if let Err(e) = r {
                info!("write error: {:?}", e);
                break;
            }
            Timer::after_secs(1).await;
        }
        // Timer::after_millis(1_000).await;
    }
}

#[task]
async fn blink_led_blue(mut led: portenta_h7_async::led::user::Blue) {
    loop {
        led.toggle();
        Timer::after_millis(1_000).await;
    }
}
