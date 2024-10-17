#![no_std]
#![no_main]
#![feature(naked_functions)]

use core::{mem::MaybeUninit, panic::PanicInfo};

use ch32_hal::exti::ExtiInput;
use ch32_hal::gpio::{Input, Pull};
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
use defmt::{info, println, trace, warn, Display2Format};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_time::{Duration, Timer};
use embassy_usb::class::hid::{HidReaderWriter, ReportId, RequestHandler, State};
use embassy_usb::control::OutResponse;
use embassy_usb::Builder;
use hal::gpio::{AnyPin, Level, Output, Pin};
use logger::set_logger;
use usbd_hid::descriptor::{KeyboardReport, SerializedDescriptor};

use bitvec::prelude as bv;

bind_interrupts!(struct Irq {
    OTG_FS => otg_fs::InterruptHandler<peripherals::OTG_FS>;
});

const DEVICE_INTERFACE_GUIDS: &[&str] = &["{DAC2087C-63FA-458D-A55D-827C0762DEC7}"];

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    critical_section::with(|_| {
        println!("{}", Display2Format(info));

        loop {}
    })
}

mod logger;

static mut LOGGER_UART: MaybeUninit<UartTx<'static, USART1, Blocking>> = MaybeUninit::uninit();

#[embassy_executor::task(pool_size = 1)]
async fn blink(pin: AnyPin, interval_ms: u64) {
    let mut led = Output::new(pin, Level::Low, Default::default());

    loop {
        led.toggle();
        Timer::after(Duration::from_millis(interval_ms)).await;
    }
}

const NR_COLS: usize = 3;
const NR_ROWS: usize = 3;

#[embassy_executor::task(pool_size = 1)]
async fn scan(cols: [AnyPin; NR_COLS], rows: [AnyPin; NR_ROWS]) {
    let mut key = bv::bitarr![u32, bv::Msb0; 0; NR_COLS * NR_ROWS];

    let mut cols: [Output; NR_COLS] =
        cols.map(|c| Output::new(c, Level::Low, ch32_hal::gpio::Speed::High));

    let rows: [Input; NR_ROWS] = rows.map(|r| Input::new(r, Pull::Down));

    loop {
        for (col, o) in cols.iter_mut().enumerate() {
            o.set_high();
            Timer::after(Duration::from_nanos(20)).await;
            for (row, i) in rows.iter().enumerate() {
                let old_level = Level::from(*key.get(col * NR_ROWS + row).unwrap());
                let new_level = i.get_level();
                if old_level != new_level {
                    trace!("<{},{}> changed to {}", row, col, new_level)
                }
                match new_level {
                    Level::Low => key.set(col * NR_ROWS + row, false),
                    Level::High => key.set(col * NR_ROWS + row, true),
                }
            }
            o.set_low();
        }
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

    let cols: [AnyPin; NR_COLS] = [p.PC6.degrade(), p.PC7.degrade(), p.PA4.degrade()];
    let rows: [AnyPin; NR_ROWS] = [p.PB1.degrade(), p.PC4.degrade(), p.PC5.degrade()];
    spawner.spawn(blink(p.PB4.degrade(), 100)).unwrap();
    spawner.spawn(scan(cols, rows)).unwrap();
    // wait for serial-cat
    Timer::after_millis(300).await;

    // Setup I2C
    let i2c_sda = p.PB11;
    let i2c_scl = p.PB10;

    let mut i2c = I2c::new_blocking(p.I2C2, i2c_scl, i2c_sda, Hertz::khz(10), Default::default());

    // let i2c_sda = p.PB9;
    // let i2c_scl = p.PB8;

    // let mut i2c = I2c::new_blocking(p.I2C1, i2c_scl, i2c_sda, Hertz::khz(10), Default::default());

    // info!("94");
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
    let mut buffer: [EndpointDataBuffer; 4] =
        core::array::from_fn(|_| EndpointDataBuffer::default());
    let driver = Driver::new(p.OTG_FS, p.PA12, p.PA11, &mut buffer);

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("HID keyboard example");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

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
    // You can also add a Microsoft OS descriptor.
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut request_handler = MyRequestHandler {};

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    // Create classes on the builder.
    let config = embassy_usb::class::hid::Config {
        report_descriptor: KeyboardReport::desc(),
        request_handler: None,
        poll_ms: 60,
        max_packet_size: 8,
    };

    let hid = HidReaderWriter::<_, 1, 8>::new(&mut builder, &mut state, config);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    let (reader, mut writer) = hid.split();

    let mut button = ExtiInput::new(p.PC13, p.EXTI13, Pull::Down);

    // Do stuff with the class!
    let in_fut = async {
        loop {
            button.wait_for_rising_edge().await;
            // signal_pin.wait_for_high().await;
            info!("Button pressed!");
            // Create a report with the A key pressed. (no shift modifier)
            let report = KeyboardReport {
                keycodes: [4, 0, 0, 0, 0, 0],
                leds: 0,
                modifier: 0,
                reserved: 0,
            };
            // Send the report.
            match writer.write_serialize(&report).await {
                Ok(()) => {}
                Err(e) => warn!("Failed to send report: {:?}", e),
            };

            button.wait_for_falling_edge().await;
            // signal_pin.wait_for_low().await;
            info!("Button released!");
            let report = KeyboardReport {
                keycodes: [0, 0, 0, 0, 0, 0],
                leds: 0,
                modifier: 0,
                reserved: 0,
            };
            match writer.write_serialize(&report).await {
                Ok(()) => {}
                Err(e) => warn!("Failed to send report: {:?}", e),
            };
        }
    };

    let out_fut = async {
        reader.run(false, &mut request_handler).await;
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join(usb_fut, join(in_fut, out_fut)).await;
    /* END USB DRIVER */
    loop {
        panic!("how are we here");
    }
}

struct MyRequestHandler {}

impl RequestHandler for MyRequestHandler {
    fn get_report(&mut self, id: ReportId, _buf: &mut [u8]) -> Option<usize> {
        info!("Get report for {:?}", id);
        None
    }

    fn set_report(&mut self, id: ReportId, data: &[u8]) -> OutResponse {
        info!("Set report for {:?}: {=[u8]}", id, data);
        OutResponse::Accepted
    }

    fn set_idle_ms(&mut self, id: Option<ReportId>, dur: u32) {
        info!("Set idle rate for {:?} to {:?}", id, dur);
    }

    fn get_idle_ms(&mut self, id: Option<ReportId>) -> Option<u32> {
        info!("Get idle rate for {:?}", id);
        None
    }
}
