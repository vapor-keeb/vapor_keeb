#![no_std]
#![no_main]

use core::{mem::MaybeUninit, panic::PanicInfo};

use async_usb_host::consts::UsbBaseClass;
use async_usb_host::descriptor::DeviceDescriptor;
use async_usb_host::request::Request;
use async_usb_host::{Host, Host2ClientMessage, HostControl, HostHandle};
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
use defmt::{info, println, trace, unwrap, warn, Display2Format};
use embassy_executor::Spawner;
use embassy_time::Timer;
use vapor_keeb::logger::set_logger;

bind_interrupts!(struct Irq {
    OTG_FS => otg_fs::InterruptHandler<peripherals::OTG_FS>;
    USBHS => usbhs::host::InterruptHandler<peripherals::USBHS>;
});

static HOST_HANDLE: HostHandle = HostHandle::new(accept_device);
static HOST_CONTROL: HostControl = HostControl::new();

const DEVICE_INTERFACE_GUIDS: &[&str] = &["{DAC2087C-63FA-458D-A55D-827C0762DEC7}"];

fn accept_device(desc: &DeviceDescriptor) -> bool {
    desc.device_class == UsbBaseClass::HID.into()
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    critical_section::with(|_| {
        println!("{}", Display2Format(info));

        loop {}
    })
}

static mut LOGGER_UART: MaybeUninit<UartTx<'static, USART1, Blocking>> = MaybeUninit::uninit();

#[embassy_executor::task]
async fn user_task(number: u8) {
    HOST_HANDLE.register().await;
    loop {
        let msg = HOST_HANDLE.recv().await;

        trace!("user: {:?}", msg);
        let dev_handle = match msg {
            Host2ClientMessage::NewDevice {
                descriptor: _,
                handle,
            } => Some(handle),
            Host2ClientMessage::ControlTransferResponse {
                result: _,
                buffer: _,
            } => {
                warn!("transfer? i hardly know her");
                None
            }
            _ => todo!(),
        };

        if let Some(dev_handle) = dev_handle {
            unwrap!(
                HOST_HANDLE
                    .control_transfer(dev_handle, Request::set_configuration(1), &mut [])
                    .await
            );
        }
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
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

    let mut host = Host::new(driver, &HOST_CONTROL, [&HOST_HANDLE]);

    spawner.must_spawn(user_task(8));

    loop {
        host.run_until_suspend().await;
    }
}
