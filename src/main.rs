#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use core::{marker::PhantomData, mem::MaybeUninit, panic::PanicInfo};

use ch32_hal::otg_fs::{Driver, EpOutBuffer};
use ch32_hal as hal;
use ch32_hal::{
    mode::Blocking,
    pac::{usart::Usart, OTG_FS, RCC},
    peripherals::{self, USART1},
    usart::{self, UartTx},
    Config, Peripheral, RccPeripheral, RemapPeripheral,
};
use defmt::{info, println, Display2Format};
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use embassy_usb::Builder;
use hal::gpio::{AnyPin, Level, Output, Pin};
use logger::set_logger;
use qingke::interrupt::Priority;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    println!("{}", Display2Format(info));

    loop {}
}

mod logger;

static mut uart: MaybeUninit<UartTx<'static, USART1, Blocking>> = MaybeUninit::uninit();

#[embassy_executor::task(pool_size = 1)]
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
    // Setup the printer
    let uart1_config = usart::Config::default();
    unsafe {
        uart = MaybeUninit::new(
            UartTx::<'static, _, _>::new_blocking(p.USART1, p.PA9, uart1_config).unwrap(),
        );
    };
    set_logger(&|data| unsafe {
        #[allow(unused_must_use, static_mut_refs)]
        uart.assume_init_mut().blocking_write(data);
    });

    spawner.spawn(blink(p.PB4.degrade(), 500)).unwrap();
    Timer::after_millis(300).await;
    info!("Starting USB");


    /* USB DRIVER SECION */
    let mut buffer: [EpOutBuffer; 4] = [EpOutBuffer::default(); 4];
    let driver = Driver::new(p.OTG_FS, p.PA12, p.PA11, &mut buffer);
    let config = embassy_usb::Config::new(0xBADF, 0xbeef);
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 7];
    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );
    let mut usb_device = builder.build();
    usb_device.run_until_suspend().await;
    /* END USB DRIVER */

    let mut next_timeout = Instant::now();
    loop {
        next_timeout += Duration::from_secs(1);
        println!("Uptime (ms): {}", Instant::now().as_millis());
        Timer::at(next_timeout).await;
    }
}
