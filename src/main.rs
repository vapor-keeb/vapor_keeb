#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use core::{marker::PhantomData, mem::MaybeUninit};

use ch32_hal::{mode::Blocking, pac::{usart::Usart, OTG_FS, RCC}, peripherals::{self, USART1}, usart::{self, UartTx}, Config, Peripheral, RccPeripheral, RemapPeripheral};
use defmt::println;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::gpio::{AnyPin, Level, Output, Pin};
use logger::set_logger;
use qingke::interrupt::Priority;
use {ch32_hal as hal, panic_halt as _};

mod logger;

static mut uart: MaybeUninit<UartTx<'static, USART1, Blocking>> = MaybeUninit::uninit();

#[embassy_executor::task(pool_size = 3)]
async fn blink(pin: AnyPin, interval_ms: u64) {
    let mut led = Output::new(pin, Level::Low, Default::default());

    loop {
        led.set_high();
        Timer::after(Duration::from_millis(interval_ms)).await;
        led.set_low();
        Timer::after(Duration::from_millis(interval_ms)).await;
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    // setup clocks
    let cfg = Config {
        rcc: ch32_hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSE,
        dma_interrupt_priority: Priority::P0,
    };
    let p = hal::init(cfg);
    hal::embassy::init();

    ::critical_section::with(|_| {
        RCC.ahbpcenr().modify(|w| w.set_otg_en(true));
        // Enable USBD

        OTG_FS.ctrl().modify(|w| {
            w.0 = 0;
            w.set_host_mode(true);
            w.set_low_speed(false);
            w.set_dev_pu_en(true);
            w.set_sys_ctrl(0b11);
        });

        OTG_FS.int_en().write(|w| w.0 = 0);
    });



    // GPIO
    spawner.spawn(blink(p.PA15.degrade(), 1000)).unwrap();
    spawner.spawn(blink(p.PB4.degrade(), 500)).unwrap();
    let cfg = usart::Config::default();
    unsafe {
        uart = MaybeUninit::new(UartTx::<'static, _, _>::new_blocking(p.USART1, p.PA9, cfg).unwrap());
    };

    set_logger(&|data| { unsafe{ uart.assume_init_mut().blocking_write(data);} });

    let mut i = 0usize;

    loop {
        Timer::after_millis(2000).await;

        println!("lol {}", i);
        i = 1 + i;
    }
}
