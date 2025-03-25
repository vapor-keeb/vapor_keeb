#![no_std]
#![no_main]

use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};
use core::{mem::MaybeUninit, panic::PanicInfo};

use async_usb_host::driver::kbd::HidKbd;
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
use defmt::error;
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
        println!("panic with regular info");
        loop {}
    })
}

static mut LOGGER_UART: MaybeUninit<UartTx<'static, USART1, Blocking>> = MaybeUninit::uninit();

/// Future for the [`select_array`] function.
#[derive(Debug)]
#[must_use = "futures do nothing unless you `.await` or poll them"]
pub struct SelectPinArray<'a, 'b, Fut, const N: usize> {
    inner: &'a mut Pin<&'b mut [Option<Fut>; N]>,
}

/// Creates a new future which will select over an array of futures.
///
/// The returned future will wait for any future to be ready. Upon
/// completion the item resolved will be returned, along with the index of the
/// future that was ready.
///
/// If the array is empty, the resulting future will be Pending forever.
pub fn select_pin_array<'a, 'b, Fut: Future, const N: usize>(
    arr: &'a mut Pin<&'b mut [Option<Fut>; N]>,
) -> SelectPinArray<'a, 'b, Fut, N> {
    SelectPinArray { inner: arr }
}

impl<'a, 'b, Fut: Future, const N: usize> Future for SelectPinArray<'a, 'b, Fut, N> {
    type Output = (Fut::Output, usize);

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // Safety: Since `self` is pinned, `inner` cannot move. Since `inner` cannot move,
        // its elements also cannot move. Therefore it is safe to access `inner` and pin
        // references to the contained futures.
        let item = unsafe { self.get_unchecked_mut().inner.as_mut().get_unchecked_mut() }
            .iter_mut()
            .enumerate()
            .find_map(|(i, f)| {
                let r = f
                    .as_mut()
                    .and_then(|f| match unsafe { Pin::new_unchecked(f) }.poll(cx) {
                        Poll::Pending => None,
                        Poll::Ready(e) => Some((i, e)),
                    });
                if let Some(_) = r {
                    *f = None;
                }
                r
            });

        match item {
            Some((idx, res)) => Poll::Ready((res, idx)),
            None => Poll::Pending,
        }
    }
}

mod driver {
    use core::{future::Future, pin::Pin};

    use super::select_pin_array;
    use arrayvec::ArrayVec;
    use async_usb_host::{
        descriptor::DeviceDescriptor, driver::kbd::HidKbd, errors::UsbHostError, pipe::USBHostPipe,
        DeviceHandle, Driver,
    };
    use defmt::{error, panic, trace, unwrap};
    use embassy_futures::select::{select, Either};
    use embassy_sync::{
        blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, mutex::Mutex,
    };

    pub struct USBHostDriver<'a, D: Driver> {
        pipe: &'a USBHostPipe<D, 16>,
        new_dev: Channel<CriticalSectionRawMutex, (DeviceHandle, DeviceDescriptor), 1>,
    }

    impl<'a, D: Driver> USBHostDriver<'a, D> {
        pub fn new(pipe: &'a USBHostPipe<D, 16>) -> Self {
            Self {
                pipe,
                new_dev: Channel::new(),
            }
        }

        pub async fn accept(
            &self,
            device: DeviceHandle,
            descriptor: DeviceDescriptor,
        ) -> Result<(), UsbHostError> {
            self.new_dev.send((device, descriptor)).await;
            Ok(())
        }

        pub async fn run<
            Fut: Future<Output = Result<(), UsbHostError>>,
            F: Fn(HidKbd<'a, D, 16>) -> Fut,
        >(
            &mut self,
            f: F,
        ) {
            let mut device_futures: [Option<Fut>; 4] = [const { None }; 4]; // TODO: const generic
            let mut pinned: Pin<&mut [Option<Fut>; 4]> =
                unsafe { Pin::new_unchecked(&mut device_futures) };

            loop {
                let new_dev_fut = self.new_dev.receive();
                match select(new_dev_fut, select_pin_array(&mut pinned)).await {
                    Either::First((device, descriptor)) => {
                        let kbd = HidKbd::try_attach(&self.pipe, device, descriptor).await;
                        match kbd {
                            Ok(kbd) => unsafe {
                                pinned.as_mut().get_unchecked_mut()[0] = Some(f(kbd));
                            },
                            Err(e) => {
                                error!("Failed to attach keyboard: {}", e);
                            }
                        }
                    }
                    Either::Second(fut) => {}
                }
            }
        }
    }
}

enum USBHostDriver {
    NoDevice,
    // Each kind of device you want to support
}

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
        let (host2, event) = host.run_until_event().await;
        host = host2;
        match event {
            async_usb_host::HostEvent::NewDevice { descriptor, handle } => {
                info!("New device {:?} with descriptor {:?}", handle, descriptor);
                match HidKbd::try_attach(&pipe, handle, descriptor).await {
                    Ok(kbd) => {
                        info!("Keyboard attached");
                    }
                    Err(e) => error!("bruh: {}", e),
                }
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
