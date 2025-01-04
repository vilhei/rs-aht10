#![no_std]

use data::Aht10Data;
/// Datasheet used for this driver : https://en.maritex.com.pl/product/attachment/147096/c5093eda6658ef654b651f3d5705c2ef
/// It's very basic, and lot of information is missing.
use embedded_hal::{
    delay::DelayNs,
    i2c::{Error, I2c},
};
// use hal::blocking::{
//     delay::DelayMs,
//     i2c::{Read, Write},
// };

pub mod data;

const AHT10_ADDRESS: u8 = 0x38;

const AHT10_INIT_MODE_NORMAL: u8 = 0x00;

const AHT10_INIT_MODE_CYCLE: u8 = 0x20;
const AHT10_INIT_MODE_CMD: u8 = 0x40; // ???

const AHT10_INIT_CALIBRATION_ON: u8 = 0x08;
const AHT10_INIT_CALIBRATION_OFF: u8 = 0x00;

enum Aht10Commands {
    Initialization = 0b1110_0001,
    TriggerMeasure = 0b1010_1100,
    SoftReset = 0b1011_1010,
}

#[derive(Debug, Copy, Clone)]
pub enum AhtError<E: Error> {
    ReadTimeout,
    BusError(E),
}

#[derive(Debug, Copy, Clone)]
pub struct Aht10Status {
    pub is_busy: bool,
    pub working_mode: u8,
    pub calibration_enable: bool,
}

pub struct AHT10<I2C> {
    i2c_dev: I2C,
}

impl<I2C> AHT10<I2C>
where
    I2C: I2c,
{
    pub fn new(i2c: I2C) -> Self {
        Self { i2c_dev: i2c }
    }

    /// Initialize the sensor.
    pub fn initialize(&mut self) -> Result<(), AhtError<I2C::Error>> {
        self.write_command(
            Aht10Commands::Initialization,
            AHT10_INIT_MODE_NORMAL | AHT10_INIT_CALIBRATION_ON,
            0x00,
        )
    }

    /// Read the internal status register.
    pub fn read_status(&mut self) -> Result<Aht10Status, AhtError<I2C::Error>> {
        let mut buffer: [u8; 6] = [0; 6];
        self.i2c_dev
            .read(AHT10_ADDRESS, &mut buffer)
            .map_err(|e| AhtError::BusError(e))?;

        let is_busy = (buffer[0] & 0x80) > 0;
        let working_mode = (buffer[0] & 0x60) >> 4;
        let calibration_enable = (buffer[0] & 0x08) > 0;

        Ok(Aht10Status {
            is_busy,
            working_mode,
            calibration_enable,
        })
    }

    /// Triggers and waits for measurements. This is a blocking function, which takes at least 70 ms (3 retries of 75ms each).
    pub fn read_data<Delay: DelayNs>(
        &mut self,
        delay: &mut Delay,
    ) -> Result<Aht10Data, AhtError<I2C::Error>> {
        self.write_command(Aht10Commands::TriggerMeasure, 0x33, 0x00)?; // 0x33 is a magic value given by the "datasheet"

        let mut status = self.read_status()?;
        for _ in 0..3 {
            if !status.is_busy {
                break;
            }

            delay.delay_ms(75);
            status = self.read_status()?;
        }

        if status.is_busy {
            return Err(AhtError::ReadTimeout);
        }

        let raw = self.read_raw_data()?;
        Ok(Aht10Data::new(raw))
    }

    /// Reset the sensor. This is a blocking function, the reset takes 20ms. WARNING: You must re-initialize the sensor after the reset !
    pub fn soft_reset<Delay: DelayNs>(
        &mut self,
        delay: &mut Delay,
    ) -> Result<(), AhtError<I2C::Error>> {
        self.write_command(Aht10Commands::SoftReset, 0, 0)?;
        delay.delay_ms(20);
        Ok(())
    }

    fn write_command(
        &mut self,
        cmd: Aht10Commands,
        data0: u8,
        data1: u8,
    ) -> Result<(), AhtError<I2C::Error>> {
        self.i2c_dev
            .write(AHT10_ADDRESS, &[cmd as u8, data0, data1])
            .map_err(|e| AhtError::BusError(e))
    }

    fn read_raw_data(&mut self) -> Result<[u8; 5], AhtError<I2C::Error>> {
        let mut buffer: [u8; 6] = [0; 6];
        self.i2c_dev
            .read(AHT10_ADDRESS, &mut buffer)
            .map_err(|e| AhtError::BusError(e))?;

        Ok(buffer[1..6].try_into().unwrap())
    }
}
