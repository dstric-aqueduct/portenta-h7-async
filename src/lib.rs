#![no_std]

pub mod led;
pub mod sys;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_stm32::{
    bind_interrupts,
    gpio::{Level, Output, Speed},
    peripherals,
    usart::{self, Uart},
    usb_otg,
    Config,
    Peripherals,
};

bind_interrupts!(pub struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
    UART4 => usart::InterruptHandler<peripherals::UART4>;
    OTG_HS => usb_otg::InterruptHandler<peripherals::USB_OTG_HS>;
});

// Naming according to breakout board
pub type Uart0 = Uart<'static, peripherals::UART4, peripherals::DMA1_CH0, peripherals::DMA1_CH1>;
pub type Uart1 = Uart<'static, peripherals::USART1, peripherals::DMA1_CH2, peripherals::DMA1_CH3>;

pub struct Board {
    pub led_red: Output<'static, peripherals::PK5>,
    pub led_green: Output<'static, peripherals::PK6>,
    pub led_blue: Output<'static, peripherals::PK7>,
    pub uart0: Uart0,
    pub uart1: Uart1,
}

impl Board {
    pub fn take() -> Self {
        static TAKEN: AtomicBool = AtomicBool::new(false);
        debug_assert!(!TAKEN.swap(true, Ordering::SeqCst));
        Self::setup()
    }

    pub fn peripherals() -> Peripherals {
        sys::Clk::new().reset().enable_ext_clock();

        let mut config = Config::default();

        {
            use embassy_stm32::rcc::*;
            config.rcc.hsi = Some(HSIPrescaler::DIV1);
            config.rcc.csi = true;
            config.rcc.hsi48 = Some(Hsi48Config { sync_from_usb: true }); // needed for USB
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

        p
    }

    pub fn setup() -> Self {
        sys::Clk::new().reset().enable_ext_clock();
        // TODO Configure 480 MHz (sys) and 240 MHz (per)
        let config = Config::default();
        let p = embassy_stm32::init(config);

        // User leds
        let led_red = Output::new(p.PK5, Level::High, Speed::Low);
        let led_green = Output::new(p.PK6, Level::High, Speed::Low);
        let led_blue = Output::new(p.PK7, Level::High, Speed::Low);

        // Uart0 of breakout board
        let uart0 = Uart::new_with_rtscts(
            p.UART4,
            p.PI9,
            p.PA0,
            Irqs,
            p.PA15,
            p.PB0,
            p.DMA1_CH0,
            p.DMA1_CH1,
            Default::default(),
        )
        .unwrap();

        // Uart1 of breakout board
        let uart1 = Uart::new_with_rtscts(
            p.USART1,
            p.PA10,
            p.PA9,
            Irqs,
            p.PA12,
            p.PA11,
            p.DMA1_CH2,
            p.DMA1_CH3,
            Default::default(),
        )
        .unwrap();

        Self {
            led_red,
            led_green,
            led_blue,
            uart0,
            uart1,
        }
    }
}
