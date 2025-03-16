#![no_std]
#![no_main]

use core::{mem::MaybeUninit, panic::PanicInfo};

use async_usb_host::pipe::USBHostPipe;
use async_usb_host::Driver;
use async_usb_host::Host;
use ch32_hal::i2c::I2c;
use ch32_hal::otg_fs::{self};
use ch32_hal::time::Hertz;
use ch32_hal::usb::EndpointDataBuffer;
use ch32_hal::usbhs::host::USBHsHostDriver;
use ch32_hal::{self as hal, bind_interrupts, peripherals, rcc, usbhs};
use ch32_hal::{
    mode::Blocking,
    peripherals::USART1,
    usart::{self, UartTx},
    Config,
};
use defmt::{info, println};
use embassy_executor::Spawner;
use embassy_time::Timer;
use vapor_keeb::logger::set_logger;

bind_interrupts!(struct Irq {
    OTG_FS => otg_fs::InterruptHandler<peripherals::OTG_FS>;
    USBHS => usbhs::host::InterruptHandler<peripherals::USBHS>;
});

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    critical_section::with(|_| {
        // println!("{}", Display2Format(info));
        println!("PANIC");

        loop {}
    })
}

static mut LOGGER_UART: MaybeUninit<UartTx<'static, USART1, Blocking>> = MaybeUninit::uninit();

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    // setup clocks
    const RCC_CFG: rcc::Config = {
        use rcc::*;

        rcc::Config {
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
    let cfg = Config {
        rcc: RCC_CFG,
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

    i2c.blocking_write(0x21, &[0x5, 0b00101011]).unwrap();
    i2c.blocking_write_read(0x21, &[0x5], &mut buf).unwrap();

    println!("0x21 0x5 reg: {:#b}", buf[0]);

    info!("Starting USB");

    let (mut a, mut b) = (EndpointDataBuffer::new(), EndpointDataBuffer::new());

    let driver = USBHsHostDriver::new(p.PB7, p.PB6, &mut a, &mut b);
    let (bus, pipe) = driver.start();
    let pipe: USBHostPipe<USBHsHostDriver<'_, _>, 16> = USBHostPipe::new(pipe);

    let mut host = Host::<'_, _, 4, 16>::new(bus, &pipe);

    loop {
        let event = host.run_until_event().await;
        match event {
            async_usb_host::HostEvent::NewDevice { descriptor, handle } => {
                info!("New device {:?} with descriptor {:?}", handle, descriptor);
            }
            async_usb_host::HostEvent::ControlTransferResponse { .. } => todo!(),
            async_usb_host::HostEvent::InterruptTransferResponse { .. } => todo!(),
            async_usb_host::HostEvent::Suspended => (),
            async_usb_host::HostEvent::DeviceDetach { mask } => {
                info!("Some device detached: {:?}", mask);
            }
        }
    }
}
