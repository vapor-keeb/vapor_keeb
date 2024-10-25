#![no_std]
#![no_main]
#![feature(naked_functions)]

use core::{mem::MaybeUninit, panic::PanicInfo};

use ch32_hal::i2c::I2c;
use ch32_hal::otg_fs::endpoint::EndpointDataBuffer;
use ch32_hal::otg_fs::{self, Driver};
use ch32_hal::time::Hertz;
use ch32_hal::{self as hal, bind_interrupts, peripherals};
use ch32_hal::{
    mode::Blocking,
    peripherals::USART1,
    usart::{self, UartTx},
    Config,
};
use consts::*;
use defmt::{debug, error, info, println, Display2Format};
use embassy_executor::Spawner;
use embassy_time::Timer;
use embassy_usb::control::{InResponse, OutResponse, Recipient, RequestType};
use embassy_usb::{Builder, Handler};
use vapor_keeb::logger::set_logger;

mod consts;

bind_interrupts!(struct Irq {
    OTG_FS => otg_fs::InterruptHandler<peripherals::OTG_FS>;
});

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    critical_section::with(|_| {
        println!("{}", Display2Format(info));

        loop {}
    })
}

static mut LOGGER_UART: MaybeUninit<UartTx<'static, USART1, Blocking>> = MaybeUninit::uninit();

const BLOCK_SIZE: usize = 32;

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    // setup clocks
    let cfg = Config {
        rcc: ch32_hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSI,
        ..Default::default()
    };
    let p = hal::init(cfg);
    hal::embassy::init();

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
    let driver = Driver::new(p.OTG_FS, p.PA12, p.PA11, &mut buffer);

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("pee pee poo poo");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0x00;
    config.device_sub_class = 0x00;
    config.device_protocol = 0x00;
    config.composite_with_iads = false;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    // You can also add a Microsoft OS descriptor.
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut request_handler = MyRequestHandler {
        buf: unsafe { core::mem::zeroed() },
        state: consts::State::DfuIdle,
        status: consts::Status::Ok,
    };

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    let mut func = builder.function(0x00, 0x00, 0x00);
    let mut iface = func.interface();
    let mut alt = iface.alt_setting(
        USB_CLASS_APPN_SPEC,
        APPN_SPEC_SUBCLASS_DFU,
        DFU_PROTOCOL_DFU,
        None,
    );
    alt.descriptor(
        DESC_DFU_FUNCTIONAL,
        &[
            DfuAttributes::empty().bits(),
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
    usb_fut.await;
    /* END USB DRIVER */
    loop {
        panic!("how are we here");
    }
}

struct MyRequestHandler {
    buf: [u8; 64],
    state: consts::State,
    status: consts::Status,
}

impl Handler for MyRequestHandler {
    fn control_out(
        &mut self,
        req: embassy_usb::control::Request,
        data: &[u8],
    ) -> Option<OutResponse> {
        if (req.request_type, req.recipient) != (RequestType::Class, Recipient::Interface) {
            return None;
        }
        debug!("{}", req);
        match Request::try_from(req.request) {
            Ok(dfu_req) => {
                match dfu_req {
                    Request::Detach => todo!(),
                    Request::Dnload => todo!(),
                    Request::Upload => todo!(),
                    Request::GetStatus => todo!(),
                    Request::ClrStatus => todo!(),
                    Request::GetState => todo!(),
                    Request::Abort => todo!(),
                }
            }
            Err(e) => error!("{}", e),
        }
        None
    }

    fn control_in<'a>(
        &'a mut self,
        req: embassy_usb::control::Request,
        buf: &'a mut [u8],
    ) -> Option<embassy_usb::control::InResponse<'a>> {
        if (req.request_type, req.recipient) != (RequestType::Class, Recipient::Interface) {
            return None;
        }
        match Request::try_from(req.request) {
            Ok(Request::GetStatus) => {
                buf[0..6].copy_from_slice(&[
                    self.status as u8,
                    0x32,
                    0x00,
                    0x00,
                    self.state as u8,
                    0x00,
                ]);

                Some(InResponse::Accepted(&buf[0..6]))
            }
            Ok(Request::Upload) => Some(InResponse::Accepted(&buf[..])),
            Ok(_) => {
                debug!("{}", req);
                None
            }
            Err(e) => {
                debug!("{}", e);
                None
            }
        }
    }
}
