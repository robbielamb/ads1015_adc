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
const ADS1X15_POINTER_LO_THRESH: u8 = 0x02;
const ADS1X15_POINTER_HI_THRESH: u8 = 0x03;

/// The following definations are for the bits in the Config Register
/// +---------+---------+---------+---------+---------+---------+---------+---------+---------+
/// |   Bit   |    15   |    14   |    13   |    12   |    11   |    10   |     9   |    8    |
/// +---------+---------+---------+---------+---------+---------+---------+---------+---------+
/// |   Name  |    OS   |   MUX2  |   MUX1  |   MUX0  |   PGA2  |   PGA1  |   PGA0  |   MODE  |
/// +---------+---------+---------+---------+---------+---------+---------+---------+---------+
///
/// +---------+---------+---------+---------+---------+---------+---------+---------+---------+
/// |   Bit   |     7   |    6    |    5    |    4    |     3   |     2   |     1   |    0    |
/// +---------+---------+---------+---------+---------+---------+---------+---------+---------+
/// |   Name  |    DR2  |    DR1  |    DR0  |COMP_MODE| COMP_POL| COMP_LAT|COMP_QUE1|COMP_QUE0|
/// +---------+---------+---------+---------+---------+---------+---------+---------+---------+
/// Location of the Operation Status bit in the config register

/// Bit [15] Operation Status Bits/Single Shot Conversion
/// This bit determins the operational status of the device.
/// Can only be written in power-down mode.
///
/// For a write status:
///  - 0: No Effect
///  - 1: Begin a single conversion (when in power down mode)
///
/// For read status:
///  - 0: Device is performing a conversion
///  - 1: Device is not performing a convirsion
const ADS1X15_CONFIG_OS_SINGLE: u16 = 0x8000;

/// Bits [14:12]
// The Offset of the PIN bits in the Config register
/// Applicable pin sets to use with the ADS1015 ADC.
/// Bits [14:12] in the Config Register
#[allow(non_camel_case_types)]
#[repr(u16)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Pin {
    /// Pins 0 and 1. Used for differental measurement.
    P0_P1 = 0b000 << ADS1X15_MUX_OFFSET,
    /// Pins 0 and 3. Used for differental measurement.
    P0_P3 = 0b001 << ADS1X15_MUX_OFFSET,
    /// Pins 1 and 3. Used for differental measurement.
    P1_p3 = 0b010 << ADS1X15_MUX_OFFSET,
    /// Pins 2 and 3. Used for differental measurement.
    P2_P3 = 0b011 << ADS1X15_MUX_OFFSET,
    // Pin 0. Used for single measurement.
    P0 = 0b100 << ADS1X15_MUX_OFFSET,
    // Pin 1. Used for single measurement.
    P1 = 0b101 << ADS1X15_MUX_OFFSET,
    // Pin 2. Used for single measurement.
    P2 = 0b110 << ADS1X15_MUX_OFFSET,
    // Pin 3. Used for single measurement.
    P3 = 0b111 << ADS1X15_MUX_OFFSET,
}
const ADS1X15_MUX_OFFSET: u16 = 12;
/// Bits [11:9]
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
// Bit 8.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum DeviceMode {
    /// Continious conversion mode
    Continuous = 0x0000,
    /// Power-down single-shot mode
    Single = 0x0100,
}

/// Bits [7:5]
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

/// Bit 4
#[repr(u16)]
#[derive(Debug)]
pub enum ComparatorMode {
    /// Traditional comparator with hystereses (default)
    Traditional = 0x0000,
    /// Window comparator
    Window = 0x0001 << 4,
}

/// Bit 3
/// Controls the polarity fo the ALERT/RDY pin. When the bit is 0,
/// the output is active low. When the bit is 1, the output is active high
#[repr(u16)]
#[derive(Copy, Clone, Debug)]
pub enum ComparatorPolarity {
    /// Active Low (default)
    ActiveLow = 0x0000,
    /// Active High
    ActiveHigh = 0x0001 << 3,
}
const COMPARATOR_BITS_MASK: u16 = 0b00001000;

/// Bit 2
/// When in continuous mode and Latching is enabled, the ALERT/READY pin will
/// remain active until the conversion register is read.
#[repr(u16)]
pub enum LatchingComparator {
    /// Non-Latching Comparator (default)
    NonLatching = 0b00,
    /// Latching Comparator
    Latching = 0b100,
}

/// Bits [1:0]
/// When the comparator function is disabled, the ALERT/RDY pin is set into a high state.
/// Other values specify the number of successive conversions exceeding the upper or
/// lower threashdolds before asserting the ALERT/RDY pin.
#[repr(u16)]
#[derive(Copy, Clone, Debug)]
pub enum ComparatorQueue {
    /// Assert after one conversion
    One = 0b00,
    /// Assert after two conversions
    Two = 0b01,
    /// Assert after four conversions
    Four = 0b10,
    /// Disable Comparator (default)
    Disable = 0b11,
}

// End Bit Definations

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

impl fmt::Display for DeviceMode {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            DeviceMode::Continuous => write!(f, "Continuous"),
            DeviceMode::Single => write!(f, "Single"),
        }
    }
}

#[derive(Debug)]
pub struct ADS1015 {
    i2cbus: i2c::I2c,
    pub data_rate: SampleRate,
    pub gain: Gain,
    pub mode: DeviceMode,
    pub polarity: ComparatorPolarity,
    comparator_queue: ComparatorQueue,
    last_pin: Option<Pin>,
}

/// TODO: Specify custom error type

type Result<T> = std::result::Result<T, i2c::Error>;

impl ADS1015 {
    /// Construct a new ADS1015 ADC on the i2c bus.
    /// Further options may be configured after it is created.
    pub fn new(i2cbus: i2c::I2c) -> Result<ADS1015> {
        let mut obj = ADS1015 {
            i2cbus,
            data_rate: SampleRate::Rate1600,
            gain: Gain::Gain1,
            mode: DeviceMode::Single,
            polarity: ComparatorPolarity::ActiveLow,
            comparator_queue: ComparatorQueue::One,
            last_pin: None,
        };

        obj.i2cbus.set_slave_address(ADS1X15_DEFAULT_ADDRESS)?;

        Ok(obj)
    }

    pub fn new_at_address(_i2cbus: i2c::I2c, _address: u16) -> Result<ADS1015> {
        // Implement once we have some custom error handling
        todo!();
    }

    /// Read the values from the given pin.
    ///
    /// The value in not adjusted and only the top 12 bit are applicable.
    fn raw_read(&mut self, pin: Pin) -> Result<u16> {
        let read_value = if self.mode == DeviceMode::Continuous && self.last_pin == Some(pin) {
            self.read_fast()?
        } else {
            self.request_read(pin)?;

            if self.mode == DeviceMode::Single {
                while !self.conversion_complete()? {
                    // Add a timeout in here?
                    continue;
                }
            }
            self.read_register(ADS1X15_POINTER_CONVERSION)?
        };

        Ok(read_value)
    }

    /// Read the value from the given pin, waiting for the conversion to complete before
    /// returning the value.
    pub fn read(&mut self, pin: Pin) -> Result<u16> {
        Ok(self.raw_read(pin)? >> 4)
    }

    /// Request a read on the specified PIN.
    ///
    /// Sends the entire config to the device, with the OS pin (15) set to 1 and
    /// the pin or pins to read.
    pub fn request_read(&mut self, pin: Pin) -> Result<()> {
        // Should this only be useable if requesting a continious mode
        // or using the alert pin?
        self.last_pin = Some(pin);
        let config = ADS1X15_CONFIG_OS_SINGLE
            | (pin as u16)
            | self.gain as u16
            | self.mode as u16
            | self.data_rate as u16
            | self.polarity as u16
            | self.comparator_queue as u16;

        // Starts the conversion
        self.write_config(config)?;
        Ok(())
    }

    /// Read the conversion register.
    ///
    /// Used when a read is requested and the ALERT/READY pin is used
    /// to determine when to read
    pub fn read_conversion(&mut self) -> Result<u16> {
        Ok(self.read_register(ADS1X15_POINTER_CONVERSION)? >> 4)
    }

    /// The number of volts the ADC is reading.
    /// Snagged from the voltage computation from:
    /// https://github.com/adafruit/Adafruit_CircuitPython_ADS1x15/blob/master/adafruit_ads1x15/analog_in.py#L77-L80
    pub fn voltage(&mut self, pin: Pin) -> Result<f32> {
        let raw = self.raw_read(pin)?;

        Ok((raw as f32) * self.gain.pga_range() / 32767.0)
    }

    /// Read the config out of the device
    /// Useful for debugging
    pub fn read_config(&mut self) -> Result<u16> {
        self.read_register(ADS1X15_POINTER_CONFIG)
    }

    fn write_config(&mut self, config: u16) -> Result<()> {
        self.write_register(ADS1X15_POINTER_CONFIG, config)?;
        Ok(())
    }

    /// Set the polarity of the Alert/Ready pin.
    pub fn set_conversion_polarity(&mut self, polarity: ComparatorPolarity) -> Result<()> {
        self.polarity = polarity;

        Ok(())
    }

    /// Check if the version is complete by reading the OS bit out of the config
    fn conversion_complete(&mut self) -> Result<bool> {
        // OS is bit 15 (0x8000)
        // OS = 0: Device is performing a conversion
        // OS = 1: Device is not performing a conversion
        let res = self.read_config()? & ADS1X15_CONFIG_OS_SINGLE;

        // res will be 0x0000 if a conversion is happening
        // and equal ADS1x15_CONFIG_OS_SINGLE if one is not
        Ok(res == ADS1X15_CONFIG_OS_SINGLE)
    }

    fn write_register(&self, reg: u8, value: u16) -> Result<()> {
        self.i2cbus.smbus_write_word_swapped(reg, value)?;
        Ok(())
    }

    /// Read a u16 from the last thing we tried to read
    fn read_fast(&mut self) -> Result<u16> {
        let buffer: &mut [u8; 2] = &mut [0; 2];

        let _read_count = self.i2cbus.read(buffer)?;

        // TODO: Should throw an error if read_count != 2

        Ok(0x0000 | (buffer[0] as u16) << 8 | buffer[1] as u16)
    }

    /// Read from a register
    fn read_register(&self, reg: u8) -> Result<u16> {
        Ok(self.i2cbus.smbus_read_word_swapped(reg)?)
    }

    /// Read the current lo thresh value
    pub fn read_lo_thresh_reg(&self) -> Result<u16> {
        Ok(self.read_register(ADS1X15_POINTER_LO_THRESH)?)
    }

    /// Read the current hi thresh value
    pub fn read_hi_thresh_reg(&self) -> Result<u16> {
        Ok(self.read_register(ADS1X15_POINTER_HI_THRESH)?)
    }

    /// Configure the ALERT/RDY pin to specify ready in single shot mode.
    /// This is done by setting max and min threshold values
    pub fn set_alert_status(&self) -> Result<()> {
        let low: u16 = 0x0000;
        let high: u16 = 0xFFFF;

        self.set_threshhold_registers(low, high)?;
        Ok(())
    }

    /// Set the threshold values default values
    pub fn set_default_threshold_values(&self) -> Result<()> {
        let low: u16 = 0x8000;
        let high: u16 = 0x7FFF;

        self.set_threshhold_registers(low, high)?;
        Ok(())
    }

    /// Set the threshold values to alert on in continuous mode.
    pub fn set_threshold_values(&self, low: u16, high: u16) -> Result<()> {
        const MAX_VALUE: u16 = 0x0FFF;
        if low >= high || low > MAX_VALUE || high > MAX_VALUE {
            // This is actually an error state
            return Ok(());
        }
        self.set_threshhold_registers(low << 4, high << 4)?;
        Ok(())
    }

    /// Helper to set the threshold registers
    fn set_threshhold_registers(&self, low: u16, high: u16) -> Result<()> {
        self.write_register(ADS1X15_POINTER_LO_THRESH, low)?;
        self.write_register(ADS1X15_POINTER_HI_THRESH, high)?;

        Ok(())
    }

    /// Sets the device back to defaults.
    /// Immediatly writes to the device and doesn't store
    /// the values for future usage.
    pub fn set_defaults(&mut self) -> Result<()> {
        self.set_default_threshold_values()?;
        let config = 0x0000
            | (Pin::P0_P1 as u16)
            | Gain::Gain2 as u16
            | DeviceMode::Single as u16
            | SampleRate::Rate1600 as u16
            | ComparatorMode::Traditional as u16
            | ComparatorPolarity::ActiveLow as u16
            | LatchingComparator::NonLatching as u16
            | ComparatorQueue::Disable as u16;

        self.write_config(config)?;
        Ok(())
    }
}

/// Makes sure the data is represented in the target. The mask specifices what
/// Bits are valid in the data
#[inline]
fn apply_bitmask_data(target: u16, data: u16, mask: u16) -> u16 {
    (target & (0xFF | mask)) ^ data
}
