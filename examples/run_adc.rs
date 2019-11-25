use rppal::i2c::I2c;
use simple_signal::{self, Signal};

use ads1015_adc::*;

use std::error::Error;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::sleep;
use std::time::Duration;

fn main() -> Result<(), Box<dyn Error>> {
    let i2c = I2c::new().expect("Unable to open I2C bus.");
    let mut adc = ADS1015::new(i2c)?;

    let running = Arc::new(AtomicBool::new(true));

    simple_signal::set_handler(&[Signal::Int, Signal::Term], {
        let r = running.clone();
        move |_signals| {
            r.store(false, Ordering::SeqCst);
        }
    });

    while running.load(Ordering::SeqCst) {
        let value = adc.read(Pin::P0)?;
        let volts = adc.voltage(Pin::P0)?;

        println!("Reading Value: {}", value);
        println!("Reading Volts: {}", volts);
        sleep(Duration::from_secs(1));
    }

    Ok(())
}
