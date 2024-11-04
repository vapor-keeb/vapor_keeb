#![no_std]
#![no_main]

use core::{mem::MaybeUninit, panic::PanicInfo};

use ch32_hal::i2c::I2c;
use ch32_hal::otg_fs::{self, Driver};
use ch32_hal::time::Hertz;
use ch32_hal::usb::EndpointDataBuffer;
use ch32_hal::{self as hal, bind_interrupts, peripherals, usbhs};
use ch32_hal::{
    mode::Blocking,
    peripherals::USART1,
    usart::{self, UartTx},
    Config,
};
use defmt::{debug, error, info, println, trace, Display2Format};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Timer;
use embassy_usb::control::{InResponse, OutResponse, Recipient, RequestType};
use embassy_usb::msos::{self, windows_version};
use embassy_usb::{Builder, Handler};
use heapless::Vec;
use usb_dfu_target::consts::{DfuAttributes, DfuRequest};
use usb_dfu_target::{DfuHandler, UsbDfuDevice};
use vapor_keeb::logger::set_logger;

bind_interrupts!(struct Irq {
    OTG_FS => otg_fs::InterruptHandler<peripherals::OTG_FS>;
    USBHS => usbhs::InterruptHandler<peripherals::USBHS>;
    USBHS_WKUP => usbhs::WakeupInterruptHandler<peripherals::USBHS>;
});

const DEVICE_INTERFACE_GUIDS: &[&str] = &["{DAC2087C-63FA-458D-A55D-827C0762DEC7}"];

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    critical_section::with(|_| {
        println!("{}", Display2Format(info));

        loop {}
    })
}

static mut LOGGER_UART: MaybeUninit<UartTx<'static, USART1, Blocking>> = MaybeUninit::uninit();

const BLOCK_SIZE: usize = 32;

struct DownloadData {
    buf: Vec<u8, 64>,
    offset: usize,
}

struct DfuDemoDevice;

impl DfuHandler for DfuDemoDevice {
    fn write_data(&mut self, offset: usize, data: &[u8]) {
        let mut v = Vec::new();
        v.extend_from_slice(data).unwrap();
        DOWNLOAD_DATA_AVAILABLE.signal(DownloadData {
            buf: v,
            offset: offset,
        });
    }

    fn complete_download(&mut self) {
        // TODO: nothing
    }

    fn upload(&self, buffer: &mut [u8], offset: usize) -> usize {
        todo!()
    }

    fn is_write_complete(&self) -> bool {
        DOWNLOAD_COMPLETE.try_take().is_some()
    }
}

static DOWNLOAD_DATA_AVAILABLE: Signal<CriticalSectionRawMutex, DownloadData> = Signal::new();
static DOWNLOAD_COMPLETE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[embassy_executor::task(pool_size = 1)]
async fn programmer() {
    loop {
        let data = DOWNLOAD_DATA_AVAILABLE.wait().await;
        info!(
            "Got data at \noffset={:x}, \ndata={:x}",
            data.offset, data.buf
        );
        DOWNLOAD_COMPLETE.signal(());
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    // setup clocks
    let cfg = Config {
        rcc: ch32_hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSI,
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

    spawner.spawn(programmer()).unwrap();
    // wait for serial-cat
    Timer::after_millis(300).await;

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

    // loop{}

    /* USB DRIVER SECION */
    let mut buffer: [EndpointDataBuffer; 1] =
        core::array::from_fn(|_| EndpointDataBuffer::default());
    let driver = usbhs::Driver::new(p.USBHS, Irq, p.PB7, p.PB6, &mut buffer);

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0x6666, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB DFU Demo");
    config.serial_number = Some("12345678");
    config.max_power = 200;
    config.max_packet_size_0 = 64;

    config.device_class = 0x00;
    config.device_sub_class = 0x00;
    config.device_protocol = 0x00;
    config.composite_with_iads = false;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 32];
    let mut bos_descriptor = [0; 64];
    let mut msos_descriptor = [0; 196];
    let mut control_buf = [0; 64];

    let mut dfu_device_handler = DfuDemoDevice;
    let mut request_handler = DfuRequestHandler {
        inner: UsbDfuDevice::new(&mut dfu_device_handler),
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
    builder.msos_feature(embassy_usb::msos::CompatibleIdFeatureDescriptor::new(
        "WINUSB", "",
    ));
    builder.msos_feature(msos::RegistryPropertyFeatureDescriptor::new(
        "DeviceInterfaceGUIDs",
        msos::PropertyData::RegMultiSz(DEVICE_INTERFACE_GUIDS),
    ));

    let mut func = builder.function(0x00, 0x00, 0x00);
    let mut iface = func.interface();
    let mut alt = {
        use usb_dfu_target::consts::*;
        iface.alt_setting(
            USB_CLASS_APPN_SPEC,
            APPN_SPEC_SUBCLASS_DFU,
            DFU_PROTOCOL_DFU,
            None,
        )
    };
    let mut attr = DfuAttributes::empty();
    attr.set(DfuAttributes::CAN_DOWNLOAD, true);
    alt.descriptor(
        usb_dfu_target::consts::DESC_DFU_FUNCTIONAL,
        &[
            attr.bits(),
            0xc4,
            0x09, // 2500ms timeout, doesn't affect operation as DETACH not necessary in bootloader code
            (BLOCK_SIZE & 0xff) as u8,
            ((BLOCK_SIZE & 0xff00) >> 8) as u8,
            0x10,
            0x01, // DFU 1.1
        ],
    );

    drop(func);
    builder.handler(&mut request_handler);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    usb_fut.await
    /* END USB DRIVER */
}

struct DfuRequestHandler<'h> {
    inner: UsbDfuDevice<'h>,
}

impl<'h> Handler for DfuRequestHandler<'h> {
    fn control_out(
        &mut self,
        req: embassy_usb::control::Request,
        buf: &[u8],
    ) -> Option<OutResponse> {
        if (req.request_type, req.recipient) != (RequestType::Class, Recipient::Interface) {
            return None;
        }

        match DfuRequest::try_from(req.request) {
            Ok(req) => match self.inner.handle_control_out(req, buf) {
                Ok(_) => Some(OutResponse::Accepted),
                Err(_) => Some(OutResponse::Rejected),
            },
            Err(_) => Some(OutResponse::Rejected),
        }
    }

    fn control_in<'a>(
        &'a mut self,
        req: embassy_usb::control::Request,
        buf: &'a mut [u8],
    ) -> Option<embassy_usb::control::InResponse<'a>> {
        if (req.request_type, req.recipient) != (RequestType::Class, Recipient::Interface) {
            return None;
        }
        match DfuRequest::try_from(req.request) {
            Ok(req) => match self.inner.handle_control_in(req, buf) {
                Ok(buf) => Some(InResponse::Accepted(buf)),
                Err(_) => Some(InResponse::Rejected),
            },
            Err(_) => Some(InResponse::Rejected),
        }
    }
}
