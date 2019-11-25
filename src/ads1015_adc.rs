//! Driver for an ADS1015 Device
//! Data Sheet: https://cdn-shop.adafruit.com/datasheets/ads1015.pdf
//!
use rppal::i2c;
//use std::error;
use std::fmt;

// TODO: Mulitple Address. The chip supports 0x48, 0x49, 0x4A, and 0x4B
const ADS1X15_DEFAULT_ADDRESS: u16 = 0x48;

// Address of the register that contains the converted value
const ADS1X15_POINTER_CONVERSION: u8 = 0x00;
// Address of the register containing the device configuration
const ADS1X15_POINTER_CONFIG: u8 = 0x01;
// Addresses containing the high and low threasholds for alerting
const _ADS1X15_POINTER_LO_THRESH: u8 = 0x02;
const _ADS1X15_POINTER_HI_THRESH: u8 = 0x03;

// Location of the Operation Status bit in the config register
const ADS1X15_CONFIG_OS_SINGLE: u16 = 0x8000;
const ADS1X15_MUX_OFFSET: u16 = 12;
const ADS1X15_COMP_QUE_DISABLE: u16 = 0x0003;

/// Applicable sample rates for the sensor.
/// Default is Rate1600
#[repr(u16)]
#[derive(Copy, Clone, Debug)]
pub enum SampleRate {
    Rate128 = 0x0000,
    Rate250 = 0x0020,
    Rate490 = 0x0040,
    Rate920 = 0x0060,
    Rate1600 = 0x0080,
    Rate2400 = 0x00A0,
    Rate3300 = 0x00C0,
}

impl fmt::Display for SampleRate {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            SampleRate::Rate128 => write!(f, "128"),
            SampleRate::Rate250 => write!(f, "250"),
            SampleRate::Rate490 => write!(f, "490"),
            SampleRate::Rate920 => write!(f, "920"),
            SampleRate::Rate1600 => write!(f, "1600"),
            SampleRate::Rate2400 => write!(f, "2400"),
            SampleRate::Rate3300 => write!(f, "3300"),
        }
    }
}

/// Applicable Gains for the sensor
/// Default is Gain2
#[repr(u16)]
#[derive(Copy, Clone, Debug)]
pub enum Gain {
    GainTwoThirds = 0x0000,
    Gain1 = 0x0200,
    Gain2 = 0x0400,
    Gain4 = 0x0600,
    Gain8 = 0x0800,
    Gain16 = 0x0A00,
}

impl fmt::Display for Gain {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Gain::GainTwoThirds => write!(f, "2/3"),
            Gain::Gain1 => write!(f, "1"),
            Gain::Gain2 => write!(f, "2"),
            Gain::Gain4 => write!(f, "4"),
            Gain::Gain8 => write!(f, "8"),
            Gain::Gain16 => write!(f, "16"),
        }
    }
}

impl Gain {
    pub fn pga_range(&self) -> f32 {
        match *self {
            Gain::GainTwoThirds => 6.166,
            Gain::Gain1 => 4.096,
            Gain::Gain2 => 2.048,
            Gain::Gain4 => 1.024,
            Gain::Gain8 => 0.512,
            Gain::Gain16 => 0.256,
        }
    }
}

/// Applicable pin sets to use with the ADS1015 ADC.
#[allow(non_camel_case_types)]
#[repr(u16)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Pin {
    /// Pins 0 and 1. Used for differental measurement.
    P0_P1 = 0b000,
    /// Pins 0 and 3. Used for differental measurement.
    P0_P3 = 0b001,
    /// Pins 1 and 3. Used for differental measurement.
    P1_p3 = 0b010,
    /// Pins 2 and 3. Used for differental measurement.
    P2_P3 = 0b011,
    // Pin 0. Used for single measurement.
    P0 = 0b100,
    // Pin 1. Used for single measurement.
    P1 = 0b101,
    // Pin 2. Used for single measurement.
    P2 = 0b110,
    // Pin 3. Used for single measurement.
    P3 = 0b111,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Mode {
    Continuous = 0x0000,
    Single = 0x0100,
}

impl fmt::Display for Mode {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Mode::Continuous => write!(f, "Continuous"),
            Mode::Single => write!(f, "Single"),
        }
    }
}

#[derive(Debug)]
pub struct ADS1015 {
    i2cbus: i2c::I2c,
    pub data_rate: SampleRate,
    pub gain: Gain,
    pub mode: Mode,
    last_pin: Option<Pin>,
}

type Result<T> = std::result::Result<T, i2c::Error>;

impl ADS1015 {
    /// Construct a new ADS1015 ADC on the i2c bus.
    /// Further options may be configured after it is created.
    pub fn new(i2cbus: i2c::I2c) -> Result<ADS1015> {
        let mut obj = ADS1015 {
            i2cbus,
            data_rate: SampleRate::Rate1600,
            gain: Gain::Gain1,
            mode: Mode::Single,
            last_pin: None,
        };

        obj.i2cbus.set_slave_address(ADS1X15_DEFAULT_ADDRESS)?;

        Ok(obj)
    }

    /// Read the values from the given pin.
    ///
    /// The value in not adjusted and only the top 12 bit are applicable.
    pub fn raw_read(&mut self, pin: Pin) -> Result<u16> {
        let read_value = if self.mode == Mode::Continuous && self.last_pin == Some(pin) {
            self.read_fast()?
        } else {
            self.last_pin = Some(pin);
            let config = ADS1X15_CONFIG_OS_SINGLE
                | (pin as u16) << ADS1X15_MUX_OFFSET
                | self.gain as u16
                | self.mode as u16
                | self.data_rate as u16
                | ADS1X15_COMP_QUE_DISABLE;

            self.write_register(ADS1X15_POINTER_CONFIG, config)?;

            if self.mode == Mode::Single {
                while !self.conversion_complete()? {
                    // Add a timeout in here?
                    continue;
                }
            }
            self.read_register(ADS1X15_POINTER_CONVERSION)?
        };

        Ok(read_value)
    }

    /// Read the value from the given pin.
    pub fn read(&mut self, pin: Pin) -> Result<u16> {
        Ok(self.raw_read(pin)? >> 4)
    }

    /// The number of volts the ADC is reading.
    /// Snagged from the voltage computation from:
    /// https://github.com/adafruit/Adafruit_CircuitPython_ADS1x15/blob/master/adafruit_ads1x15/analog_in.py
    pub fn voltage(&mut self, pin: Pin) -> Result<f32> {
        let raw = self.raw_read(pin)?;

        Ok((raw as f32) * self.gain.pga_range() / 32767.0)
    }

    /// Read the config out of the device
    pub fn read_config(&mut self) -> Result<u16> {
        self.read_register(ADS1X15_POINTER_CONFIG)
    }

    fn conversion_complete(&self) -> Result<bool> {
        // OS is bit 15 (0x8000)
        // OS = 0: Device is performing a conversion
        // OS = 1: Device is not performing a conversion
        let res = self.read_register(ADS1X15_POINTER_CONFIG)? & ADS1X15_CONFIG_OS_SINGLE;

        // res will be 0x0000 if a conversion is happening
        // and equal ADS1x15_CONFIG_OS_SINGLE if one is not
        Ok(res == ADS1X15_CONFIG_OS_SINGLE)
    }

    fn write_register(&self, reg: u8, value: u16) -> Result<()> {
        self.i2cbus.smbus_write_word_swapped(reg, value)?;
        Ok(())
    }

    /// Read a long from the last thing we tried to read
    fn read_fast(&mut self) -> Result<u16> {
        let buffer: &mut [u8; 2] = &mut [0; 2];
        //let mut buffer: [u8];

        let _read_count = self.i2cbus.read(buffer)?;

        // Should throw an error if read_count != 2

        Ok(0x0000 | (buffer[0] as u16) << 8 | buffer[1] as u16)
    }

    /// Read from a register
    fn read_register(&self, reg: u8) -> Result<u16> {
        Ok(self.i2cbus.smbus_read_word_swapped(reg)?)
    }
}
