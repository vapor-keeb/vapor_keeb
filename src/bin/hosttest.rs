#![no_std]
#![no_main]

use core::future::Future;
use core::pin::{pin, Pin};
use core::task::{Context, Poll};
use core::{mem::MaybeUninit, panic::PanicInfo};

use async_usb_host::driver::kbd::HidKbd;
use async_usb_host::errors::UsbHostError;
use async_usb_host::futures::SelectPin2;
use async_usb_host::pipe::USBHostPipe;
use async_usb_host::Host;
use async_usb_host::HostDriver;
use ch32_hal::i2c::I2c;
use ch32_hal::otg_fs::{self};
use ch32_hal::time::Hertz;
use ch32_hal::usb::EndpointDataBuffer;
use ch32_hal::usbhs::host::USBHsHostDriver;
use ch32_hal::{self as hal, bind_interrupts, gpio, peripherals, rcc, usbhs, Peripherals};
use ch32_hal::{
    mode::Blocking,
    peripherals::USART1,
    usart::{self, UartTx},
    Config,
};
use defmt::error;
use defmt::{info, println};
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_time::Timer;
use vapor_keeb::logger::{self, set_logger};

bind_interrupts!(struct Irq {
    OTG_FS => otg_fs::InterruptHandler<peripherals::OTG_FS>;
    USBHS => usbhs::host::InterruptHandler<peripherals::USBHS>;
});

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    critical_section::with(|_| {
        if unsafe { !vapor_keeb::logger::has_logger() } {
            let uart1_config = usart::Config::default();
            unsafe {
                // SAFETY: PANICCCCCCCC
                let p = Peripherals::steal();
                LOGGER_UART = MaybeUninit::new(
                    UartTx::<'static, _, _>::new_blocking(p.USART1, p.PA9, uart1_config).unwrap(),
                );
            };
            set_logger(&|data| unsafe {
                #[allow(unused_must_use, static_mut_refs)]
                LOGGER_UART.assume_init_mut().blocking_write(data);
            });
        }
        let info = unsafe { core::ptr::read_volatile(&raw const info) };
        if let Some(location) = info.location() {
            println!(
                "panic occurred in file '{}' at line {}",
                location.file(),
                location.line()
            );
        } else {
            println!("panic occurred but can't get location information...");
        }
        loop {}
    })
}

static mut LOGGER_UART: MaybeUninit<UartTx<'static, USART1, Blocking>> = MaybeUninit::uninit();

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let rcc_cfg: rcc::Config = {
        use rcc::*;

        Config {
            hse: Some(Hse {
                freq: Hertz(16_000_000),
                mode: HseMode::Oscillator,
            }),
            sys: Sysclk::PLL,
            pll_src: PllSource::HSE,
            pll: Some(Pll {
                prediv: PllPreDiv::DIV2,
                mul: PllMul::MUL18,
            }),
            pllx: None,
            ahb_pre: AHBPrescaler::DIV1,
            apb1_pre: APBPrescaler::DIV1,
            apb2_pre: APBPrescaler::DIV1,
            ls: LsConfig::default_lsi(),
            hspll_src: HsPllSource::HSE,
            hspll: Some(HsPll {
                pre: HsPllPrescaler::DIV4,
            }),
        }
    };

    // setup clocks
    let cfg = Config {
        rcc: rcc_cfg,
        ..Default::default()
    };
    let p = hal::init(cfg);

    // Setup the printer
    let uart1_config = usart::Config::default();
    unsafe {
        LOGGER_UART = MaybeUninit::new(
            UartTx::<'static, _, _>::new_blocking(p.USART1, p.PA9, uart1_config).unwrap(),
        );
    };
    set_logger(&|data| unsafe {
        #[allow(unused_must_use, static_mut_refs)]
        LOGGER_UART.assume_init_mut().blocking_write(data);
    });

    // wait for serial-cat
    Timer::after_millis(2069).await;

    // Setup I2C
    let i2c_sda = p.PB11;
    let i2c_scl = p.PB10;

    let mut i2c = I2c::new_blocking(
        p.I2C2,
        i2c_scl,
        i2c_sda,
        Hertz::khz(100),
        Default::default(),
    );

    let mut buf = [0u8; 1];
    i2c.blocking_write(0x31, &[0x5, 0b00101011]).unwrap();
    i2c.blocking_write_read(0x31, &[0x5], &mut buf).unwrap();
    println!("0x31 0x5 reg: {:#b}", buf[0]);
    i2c.blocking_write(0x31, &[0x03, 0b0100_0111]);

    i2c.blocking_write(0x21, &[0x5, 0b00101011]).unwrap();
    i2c.blocking_write_read(0x21, &[0x5], &mut buf).unwrap();
    println!("0x21 0x5 reg: {:#b}", buf[0]);
    i2c.blocking_write(0x21, &[0x03, 0b0100_0111]);

    let sp: usize;
    unsafe { core::arch::asm!("mv {}, sp", out(reg) sp) };
    info!("sp is at {:#x}", sp);
    info!("Starting USB");

    let (mut a, mut b) = (EndpointDataBuffer::new(), EndpointDataBuffer::new());

    const NR_DEVICES: usize = 16;
    let driver = USBHsHostDriver::new(p.PB7, p.PB6, &mut a, &mut b);
    let (bus, pipe) = driver.start();
    let pipe: USBHostPipe<USBHsHostDriver<'_, _>, NR_DEVICES> = USBHostPipe::new(pipe);

    // Create our host driver with support for multiple devices
    let mut host_dispatcher =
        async_usb_host::driver::USBDeviceDispatcher::<HidKbd, _, NR_DEVICES>::new(&pipe);
    let mut host = Host::<'_, _, 4, NR_DEVICES>::new(bus, &pipe);

    // Create the futures for host_driver and host
    let host_dispatcher_fut = host_dispatcher.run();
    let host_fut = host.run_until_event();

    // Create and pin the SelectPin2 instance
    let mut select = pin!(SelectPin2::with_futures(host_dispatcher_fut, host_fut));

    loop {
        match select.as_mut().await {
            Either::First(_) => {
                // host_driver exited, which shouldn't happen in normal operation
                error!("USB Host driver exited unexpectedly");
                break; // Exit the loop as requested
            }
            Either::Second((new_host, event)) => {
                // Update the host with the new state
                host = new_host;

                // Handle the event
                match event {
                    async_usb_host::HostEvent::NewDevice { descriptor, handle } => {
                        info!("New device {:?} with descriptor {:?}", handle, descriptor);
                        // Send directly to the channel instead of using accept
                        host_dispatcher.insert_new_device(handle, descriptor).await;
                    }
                    async_usb_host::HostEvent::ControlTransferResponse { .. } => todo!(),
                    async_usb_host::HostEvent::InterruptTransferResponse { .. } => todo!(),
                    async_usb_host::HostEvent::Suspended => (),
                    async_usb_host::HostEvent::DeviceDetach { mask } => {
                        info!("Some device detached: {:?}", mask);
                    }
                }

                // Create a new future for the host and insert it into the selector
                let new_host_fut = host.run_until_event();
                defmt::unwrap!(select.as_mut().insert_fut2(new_host_fut));
            }
        }
    }

    // We should never reach here, but if we do, loop forever
    loop {
        error!("USB Host system has stopped!");
        Timer::after_secs(1).await;
    }
}
