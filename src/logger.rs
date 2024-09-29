use core::sync::atomic::{AtomicBool, Ordering};

use critical_section::RestoreState;
use defmt::Encoder;

static mut encoder: Encoder = Encoder::new();
static logger_acquired: AtomicBool = AtomicBool::new(false);
static mut restore_state: critical_section::RestoreState = RestoreState::invalid();

static mut uart_tx: Option<&'static dyn Fn(&[u8]) -> ()> = None;

pub fn set_logger(write: &'static dyn Fn(&[u8]) -> ()) {
    unsafe { uart_tx = Some(write) }
}

fn uart_tx_write(bytes: &[u8]) {
    unsafe { uart_tx.map(|write| write(bytes)) };
}

#[defmt::global_logger]
pub struct Logger;
unsafe impl defmt::Logger for Logger {
    fn acquire() {
        let cs_handle = unsafe { critical_section::acquire() };
        let acquired = logger_acquired.load(Ordering::Acquire);
        if acquired {
            // panic equivalent to avoid nesting
            loop {}
        }
        unsafe {
            restore_state = cs_handle;
        }
        logger_acquired.store(true, Ordering::Release);
        unsafe { encoder.start_frame(uart_tx_write) };
    }

    unsafe fn flush() {
    }

    unsafe fn release() {
        encoder.end_frame(uart_tx_write);
        let acquired = logger_acquired.load(Ordering::Acquire);
        if !acquired {
            // panic equivalent to avoid nesting
            loop {}
        }
        critical_section::release(restore_state);
        logger_acquired.store(false, Ordering::Release);
    }

    unsafe fn write(bytes: &[u8]) {
        encoder.write(bytes, uart_tx_write);
    }
}