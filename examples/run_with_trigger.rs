use rppal::i2c::I2c;
use rppal::gpio::{Gpio, Trigger};
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

    let gpio = Gpio::new()?;
    let mut pin = gpio.get(16)?.into_input_pullup();

    let running = Arc::new(AtomicBool::new(true));

    simple_signal::set_handler(&[Signal::Int, Signal::Term], {
        let r = running.clone();
        move |_signals| {
            r.store(false, Ordering::SeqCst);
        }
    });

    
    adc.set_conversion_polarity(ComparatorPolarity::ActiveLow)?;

    let low_thresh = adc.read_lo_thresh_reg()?;
    let high_thresh = adc.read_hi_thresh_reg()?;

    println!("Low : {:X} {:b}", low_thresh, low_thresh);
    println!("High: {:X} {:b}", high_thresh, high_thresh);

    adc.set_alert_status()?;

    let low_thresh = adc.read_lo_thresh_reg()?;
    let high_thresh = adc.read_hi_thresh_reg()?;

    println!("Low : {:X} {:b}", low_thresh, low_thresh);
    println!("High: {:X} {:b}", high_thresh, high_thresh);

    pin.set_interrupt(Trigger::Both)?;
    println!("Pin Level is starting at{:?}",pin.read());

    while running.load(Ordering::SeqCst) {
        let start_val = std::time::Instant::now();
        //println!("Pin Level is starting at{:?}",pin.read());
         adc.request_read(Pin::P0)?;
        
        if let Ok(trig) = pin.poll_interrupt(true, Some(Duration::from_secs(3))) {
            println!("Triggered level: {:?}", trig);
            let value = adc.read_conversion()?;
             let read_duration = start_val.elapsed();
            println!("Reading Value: {} in {}", value, read_duration.as_micros());
        } else {
            println!("Timed out");
            println!("Pin Level is at {:?}",pin.read());
        } 
       
        sleep(Duration::from_secs(1));
    }

    println!("Pin Level Ending at {:?}",pin.read());

    adc.set_defaults()?;

    Ok(())
}