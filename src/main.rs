#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use core::{mem::MaybeUninit, panic::PanicInfo};

use ch32_hal::otg_fs::{self, Driver, EndpointDataBuffer};
use ch32_hal::{self as hal};
use ch32_hal::{
    mode::Blocking,
    peripherals::USART1,
    usart::{self, UartTx},
    Config,
};
use defmt::{info, println, Display2Format};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_time::{Duration, Instant, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::control::{InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::msos::{self, windows_version};
use embassy_usb::types::InterfaceNumber;
use embassy_usb::{Builder, Handler};
use embassy_usb_driver::EndpointError;
use hal::gpio::{AnyPin, Level, Output, Pin};
use logger::set_logger;

const DEVICE_INTERFACE_GUIDS: &[&str] = &["{DAC2087C-63FA-458D-A55D-827C0762DEC7}"];

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    critical_section::with(|_| {
        println!("{}", Display2Format(info));

        loop {}
    })
}

mod logger;

static mut uart: MaybeUninit<UartTx<'static, USART1, Blocking>> = MaybeUninit::uninit();

#[embassy_executor::task(pool_size = 1)]
async fn blink(pin: AnyPin, interval_ms: u64) {
    let mut led = Output::new(pin, Level::Low, Default::default());

    loop {
        led.toggle();
        Timer::after(Duration::from_millis(interval_ms)).await;
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    // setup clocks
    let cfg = Config {
        rcc: ch32_hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSE,
        ..Default::default()
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

    spawner.spawn(blink(p.PB4.degrade(), 100)).unwrap();
    Timer::after_millis(300).await;
    info!("Starting USB");

    /* USB DRIVER SECION */
    let mut buffer: [EndpointDataBuffer; 4] = [EndpointDataBuffer::default(); 4];
    let driver = Driver::new(p.OTG_FS, p.PA12, p.PA11, &mut buffer);
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-raw");
    config.serial_number = Some("12345678");

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut handler = ControlHandler {
        if_num: InterfaceNumber(0),
    };

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );



    builder.msos_descriptor(windows_version::WIN8_1, 0);
    builder.msos_feature(msos::CompatibleIdFeatureDescriptor::new("WINUSB", ""));
    builder.msos_feature(msos::RegistryPropertyFeatureDescriptor::new(
        "DeviceInterfaceGUIDs",
        msos::PropertyData::RegMultiSz(DEVICE_INTERFACE_GUIDS),
    ));


    // Add a vendor-specific function (class 0xFF), and corresponding interface,
    // that uses our custom handler.
    let mut function = builder.function(0xFF, 0, 0);
    let mut interface = function.interface();
    let _alternate = interface.alt_setting(0xFF, 0, 0, None);
    handler.if_num = interface.interface_number();
    drop(function);
    builder.handler(&mut handler);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    // usb.run().await;

    // let mut builder = Builder::new(
    //     driver,
    //     config,
    //     &mut config_descriptor,
    //     &mut bos_descriptor,
    //     &mut [], // no msos descriptors
    //     &mut control_buf,
    // );
    // let mut usb_device = builder.build();
    loop {
        usb.run_until_suspend().await;
        println!("USB Suspended at: {}ms", Instant::now().as_millis());
        usb.wait_resume().await;
        println!("USB Resumed at: {}ms", Instant::now().as_millis());
    }
    /* END USB DRIVER */
    loop {
        panic!("how are we here");
    }
}

/// Handle CONTROL endpoint requests and responses. For many simple requests and responses
/// you can get away with only using the control endpoint.
struct ControlHandler {
    if_num: InterfaceNumber,
}

impl Handler for ControlHandler {
    /// Respond to HostToDevice control messages, where the host sends us a command and
    /// optionally some data, and we can only acknowledge or reject it.
    fn control_out<'a>(&'a mut self, req: Request, buf: &'a [u8]) -> Option<OutResponse> {
        // Log the request before filtering to help with debugging.
        info!("Got control_out, request={}, buf={:a}", req, buf);

        // Only handle Vendor request types to an Interface.
        if req.request_type != RequestType::Vendor || req.recipient != Recipient::Interface {
            return None;
        }

        // Ignore requests to other interfaces.
        if req.index != self.if_num.0 as u16 {
            return None;
        }

        // Accept request 100, value 200, reject others.
        if req.request == 100 && req.value == 200 {
            Some(OutResponse::Accepted)
        } else {
            Some(OutResponse::Rejected)
        }
    }

    /// Respond to DeviceToHost control messages, where the host requests some data from us.
    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        info!("Got control_in, request={}", req);

        // Only handle Vendor request types to an Interface.
        if req.request_type != RequestType::Vendor || req.recipient != Recipient::Interface {
            return None;
        }

        // Ignore requests to other interfaces.
        if req.index != self.if_num.0 as u16 {
            return None;
        }

        // Respond "hello" to request 101, value 201, when asked for 5 bytes, otherwise reject.
        if req.request == 101 && req.value == 201 && req.length == 5 {
            buf[..5].copy_from_slice(b"hello");
            Some(InResponse::Accepted(&buf[..5]))
        } else {
            Some(InResponse::Rejected)
        }
    }
}
