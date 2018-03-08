//! Inter-Integrated Circuit (I2C) bus

use cast::{u8, u16, u32};
use stm32f429::{I2C1, I2C2, I2C3};

use gpio::gpioa::{PA10, PA9};
use gpio::gpiob::{PB6, PB7, PB8, PB9};
use gpio::gpiof::{PF0, PF1, PF6};
use gpio::AF4;
use hal::blocking::i2c::{Write, WriteRead};
use rcc::{APB1, Clocks};
use time::Hertz;

/// I2C error
#[derive(Debug)]
pub enum Error {
    /// Bus error
    Bus,
    /// Arbitration loss
    Arbitration,
    // Overrun, // slave mode only
    // Pec, // SMBUS mode only
    // Timeout, // SMBUS mode only
    // Alert, // SMBUS mode only
    #[doc(hidden)] _Extensible,
}

// FIXME these should be "closed" traits
/// SCL pin -- DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait SclPin<I2C> {}

/// SDA pin -- DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait SdaPin<I2C> {}

// unsafe impl SclPin<I2C1> for PA15<AF4> {}
unsafe impl SclPin<I2C1> for PB6<AF4> {}
unsafe impl SclPin<I2C1> for PB8<AF4> {}

unsafe impl SclPin<I2C2> for PA9<AF4> {}
unsafe impl SclPin<I2C2> for PF1<AF4> {}
unsafe impl SclPin<I2C2> for PF6<AF4> {}

// unsafe impl SdaPin<I2C1> for PA14<AF4> {}
unsafe impl SdaPin<I2C1> for PB7<AF4> {}
unsafe impl SdaPin<I2C1> for PB9<AF4> {}

unsafe impl SdaPin<I2C2> for PA10<AF4> {}
unsafe impl SdaPin<I2C2> for PF0<AF4> {}

/// I2C peripheral operating in master mode
pub struct I2c<I2C, PINS> {
    i2c: I2C,
    pins: PINS,
}

macro_rules! busy_wait {
    ($i2c:expr, $flag:ident) => {
        loop {
            let isr = $i2c.sr1.read();

            if isr.berr().bit_is_set() {
                return Err(Error::Bus);
            } else if isr.arlo().bit_is_set() {
                return Err(Error::Arbitration);
            } else if isr.$flag().bit_is_set() {
                break;
            } else {
                // try again
            }
        }
    }
}

macro_rules! hal {
    ($($I2CX:ident: ($i2cX:ident, $i2cXen:ident, $i2cXrst:ident),)+) => {
        $(
            impl<SCL, SDA> I2c<$I2CX, (SCL, SDA)> {
                /// Configures the I2C peripheral to work in master mode
                pub fn $i2cX<F>(
                    i2c: $I2CX,
                    pins: (SCL, SDA),
                    freq: F,
                    clocks: Clocks,
                    apb1: &mut APB1,
                ) -> Self where
                    F: Into<Hertz>,
                    SCL: SclPin<$I2CX>,
                    SDA: SdaPin<$I2CX>,
                {
                    apb1.enr().modify(|_, w| w.$i2cXen().set_bit());
                    apb1.rstr().modify(|_, w| w.$i2cXrst().set_bit());
                    apb1.rstr().modify(|_, w| w.$i2cXrst().clear_bit());

                    let i2cclk = clocks.pclk1().0;
                    let freq = freq.into().0;
                    let freq_mhz = u8(freq / 1_000_000).unwrap();
                    i2c.cr2.modify(|_, w| unsafe { w.freq().bits(freq_mhz) });
                    // T[high] = T[low] = CCR * T[pclk1]
                    let ccr = u16(i2cclk / freq).unwrap();
                    i2c.ccr.modify(|_, w| unsafe { w.ccr().bits(ccr) });
                    let trise = u8(i2cclk / freq + 1).unwrap();
                    i2c.trise.modify(|_, w| unsafe { w.trise().bits(trise) });

                    // Enable the peripheral
                    i2c.cr1.write(|w| w.pe().set_bit());

                    I2c { i2c, pins }
                }

                /// Releases the I2C peripheral and associated pins
                pub fn free(self) -> ($I2CX, (SCL, SDA)) {
                    (self.i2c, self.pins)
                }
            }

            impl<PINS> Write for I2c<$I2CX, PINS> {
                type Error = Error;

                fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
                    while self.i2c.sr2.read().busy().bit() {}
                    // START
                    self.i2c.cr1.write(|w| {
                        w.start().set_bit()
                    });
                    // Wait for master mode selected
                    while self.i2c.sr1.read().sb().bit() ||
                          self.i2c.sr2.read().msl().bit() ||
                          self.i2c.sr2.read().busy().bit() {}
                    // Send address
                    self.i2c.dr.write(|w| unsafe { w.bits((u32(addr) << 1) | 1) });
                    // Wait for address sent
                    busy_wait!(self.i2c, addr);
                    // while self.i2c.cr1().read().addr().bits() != 0 {}

                    for byte in bytes {
                        // Wait until we are allowed to send data (START has been ACKed or last byte
                        // when through)
                        busy_wait!(self.i2c, btf);

                        // put byte on the wire
                        self.i2c.dr.write(|w| unsafe { w.bits(u32(*byte)) });
                    }

                    // Wait until the last transmission is finished ???
                    // busy_wait!(self.i2c, busy);

                    // automatic STOP

                    Ok(())
                }
            }

            impl<PINS> WriteRead for I2c<$I2CX, PINS> {
                type Error = Error;

                fn write_read(
                    &mut self,
                    addr: u8,
                    bytes: &[u8],
                    buffer: &mut [u8],
                ) -> Result<(), Error> {
                    self.write(addr, bytes)?;

                    // Wait until the last transmission is finished
                    busy_wait!(self.i2c, btf);

                    // reSTART and prepare to receive bytes into `buffer`
                    self.i2c.cr1.write(|w| {
                        w.start().set_bit()
                            .ack().set_bit()
                    });
                    // Wait for master mode selected
                    while self.i2c.sr1.read().sb().bit() ||
                          self.i2c.sr2.read().msl().bit() ||
                          self.i2c.sr2.read().busy().bit() {}
                    // Send address
                    self.i2c.dr.write(|w| unsafe { w.bits((u32(addr) << 1) | 0) });
                    // Wait for address sent
                    busy_wait!(self.i2c, addr);

                    let len = buffer.len();
                    for (i, byte) in buffer.iter_mut().enumerate() {
                        if i == len - 1 {
                            self.i2c.cr1.write(|w| {
                                w.ack().clear_bit()
                            });
                        }
                        // Wait until we have received something
                        busy_wait!(self.i2c, rx_ne);

                        *byte = self.i2c.dr.read().bits() as u8;
                    }

                    // automatic STOP

                    Ok(())
                }
            }
        )+
    }
}

hal! {
    I2C1: (i2c1, i2c1en, i2c1rst),
    I2C2: (i2c2, i2c2en, i2c2rst),
    I2C3: (i2c3, i2c3en, i2c3rst),
}
