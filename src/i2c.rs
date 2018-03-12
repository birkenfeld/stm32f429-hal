//! Inter-Integrated Circuit (I2C) bus

use cast::{u8, u16, u32};
use stm32f429::{I2C1, I2C2, I2C3};

use gpio::gpioa::PA8;
use gpio::gpiob::{PB6, PB7, PB8, PB9, PB10, PB11};
use gpio::gpioc::{PC9};
use gpio::gpiof::{PF0, PF1};
use gpio::gpioh::{PH4, PH5, PH7, PH8};
use gpio::AF4;
use hal::blocking::i2c::{Read, Write};
use rcc::{APB1, Clocks};
use time::{KiloHertz, MegaHertz};


/// I2C error
#[derive(Debug)]
pub enum Error {
    /// Bus error
    Bus,
    /// Arbitration loss
    Arbitration,
    /// No acknowledge returned
    Acknowledge,
    /// Overrun/Underrun, slave mode only
    Overrun,
    /// Packet Error Checking corruption, SMBUS mode only
    Pec,
    /// SMBUS mode only
    Timeout,
    /// SMBUS mode only
    Alert,
    #[doc(hidden)] _Extensible,
}

// FIXME these should be "closed" traits
/// SCL pin -- DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait SclPin<I2C> {}

/// SDA pin -- DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait SdaPin<I2C> {}

unsafe impl SclPin<I2C1> for PB6<AF4> {}
unsafe impl SclPin<I2C1> for PB8<AF4> {}

unsafe impl SclPin<I2C2> for PB10<AF4> {}
unsafe impl SclPin<I2C2> for PF1<AF4> {}
unsafe impl SclPin<I2C2> for PH4<AF4> {}

unsafe impl SclPin<I2C3> for PA8<AF4> {}
unsafe impl SclPin<I2C3> for PH7<AF4> {}

unsafe impl SdaPin<I2C1> for PB7<AF4> {}
unsafe impl SdaPin<I2C1> for PB9<AF4> {}

unsafe impl SdaPin<I2C2> for PB11<AF4> {}
unsafe impl SdaPin<I2C2> for PF0<AF4> {}
unsafe impl SdaPin<I2C2> for PH5<AF4> {}

unsafe impl SdaPin<I2C3> for PC9<AF4> {}
unsafe impl SdaPin<I2C3> for PH8<AF4> {}

/// I2C peripheral operating in master mode
pub struct I2c<I2C, SCL, SDA> {
    i2c: I2C,
    scl: SCL,
    sda: SDA,
}

macro_rules! busy_wait {
    ($i2c:expr, $flag:ident) => {
        loop {
            let isr = $i2c.sr1.read();

            if isr.$flag().bit_is_set() {
                break;
            } else if isr.berr().bit_is_set() {
                return Err(Error::Bus);
            } else if isr.arlo().bit_is_set() {
                return Err(Error::Arbitration);
            } else if isr.af().bit_is_set() {
                return Err(Error::Acknowledge);
            } else if isr.ovr().bit_is_set() {
                return Err(Error::Overrun);
            } else if isr.pecerr().bit_is_set() {
                return Err(Error::Pec);
            } else if isr.timeout().bit_is_set() {
                return Err(Error::Timeout);
            } else if isr.smbalert().bit_is_set() {
                return Err(Error::Alert);
            } else {
                $i2c.sr2.read();
                // try again
            }
        }
    }
}

macro_rules! hal {
    ($($I2CX:ident: ($i2cX:ident, $i2cXen:ident, $i2cXrst:ident),)+) => {
        $(
            impl<SCL, SDA> I2c<$I2CX, SCL, SDA> {
                /// Configures the I2C peripheral to work in master mode
                ///
                /// freq: Maximum 400 KHz
                pub fn $i2cX<F>(
                    i2c: $I2CX,
                    scl: SCL,
                    sda: SDA,
                    freq: F,
                    clocks: Clocks,
                    apb1: &mut APB1,
                ) -> Self where
                    F: Into<KiloHertz>,
                    SCL: SclPin<$I2CX>,
                    SDA: SdaPin<$I2CX>,
                {
                    // Enable peripheral
                    apb1.enr().modify(|_, w| w.$i2cXen().set_bit());
                    // Reset peripheral
                    apb1.rstr().modify(|_, w| w.$i2cXrst().set_bit());
                    apb1.rstr().modify(|_, w| w.$i2cXrst().clear_bit());

                    let i2cclk = clocks.pclk1();
                    let i2cclk_mhz = Into::<MegaHertz>::into(i2cclk).0;
                    let i2cfreq_mhz = u8(i2cclk_mhz).unwrap()
                        .max(2)    // Minimum 2 MHz
                        .min(50);  // Maximum 50 MHz
                    i2c.cr2.modify(|_, w| unsafe { w.freq().bits(i2cfreq_mhz) });
                    // T[high] = T[low] = CCR * T[pclk1]
                    let i2cclk_khz = Into::<KiloHertz>::into(i2cclk).0;
                    let freq_khz = freq.into().0;
                    let ccr = u16((i2cclk_khz / freq_khz) >> 1).unwrap();
                    i2c.ccr.modify(|_, w| unsafe { w.ccr().bits(ccr) });
                    let trise = u8(((i2cclk_khz / freq_khz) >> 1) + 1).unwrap();
                    i2c.trise.modify(|_, w| unsafe { w.trise().bits(trise) });

                    // Enable the peripheral
                    i2c.cr1.write(|w| w.pe().set_bit());

                    I2c { i2c, scl, sda }
                }

                /// Releases the I2C peripheral and associated pins
                pub fn free(self) -> ($I2CX, SCL, SDA) {
                    (self.i2c, self.scl, self.sda)
                }

                fn wait_busy(&self) {
                    while self.i2c.sr2.read().busy().bit() {}
                }
            }

            impl<SCL, SDA> Read for I2c<$I2CX, SCL, SDA> {
                type Error = Error;

                fn read(
                    &mut self,
                    addr: u8,
                    buffer: &mut [u8],
                ) -> Result<(), Error> {
                    // START
                    self.i2c.cr1.modify(|_, w| {
                        w.start().set_bit()
                            .ack().set_bit()
                    });
                    // Wait for master mode selected
                    while ! self.i2c.sr1.read().sb().bit() ||
                          ! self.i2c.sr2.read().msl().bit() {}
                    // Send address
                    self.i2c.dr.write(|w| unsafe { w.bits((u32(addr) << 1) | 1) });
                    // Wait for address sent
                    busy_wait!(self.i2c, addr);
                    // To clear addr
                    self.i2c.sr2.read();

                    let len = buffer.len();
                    for (i, byte) in buffer.iter_mut().enumerate() {
                        if i == len - 1 {
                            self.i2c.cr1.modify(|_, w| {
                                w.ack().clear_bit()
                            });
                        } else {
                        }

                        // Wait until we have received something
                        busy_wait!(self.i2c, rx_ne);

                        *byte = self.i2c.dr.read().bits() as u8;
                    }
                    // STOP
                    self.i2c.cr1.modify(|_, w| {
                        w.stop().set_bit()
                    });
                    while self.i2c.sr2.read().busy().bit() {
                        if self.i2c.sr1.read().rx_ne().bit() {
                            self.i2c.dr.read();
                        }
                    }

                    Ok(())
                }
            }

            impl<SCL, SDA> Write for I2c<$I2CX, SCL, SDA> {
                type Error = Error;

                fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
                    // START
                    self.i2c.cr1.modify(|_, w| {
                        w.start().set_bit()
                            .ack().set_bit()
                    });
                    // Wait for master mode selected
                    while ! self.i2c.sr1.read().sb().bit() ||
                          ! self.i2c.sr2.read().msl().bit() {}
                    // Send address
                    self.i2c.dr.write(|w| unsafe { w.bits((u32(addr) << 1) | 0) });
                    // Wait for address sent
                    busy_wait!(self.i2c, addr);
                    // To clear addr
                    self.i2c.sr2.read();
                    
                    for byte in bytes {
                        // Wait until we are allowed to send data (START has been ACKed or last byte
                        // when through)
                        busy_wait!(self.i2c, tx_e);

                        // put byte on the wire
                        self.i2c.dr.write(|w| unsafe { w.bits(u32(*byte)) });
                    }

                    // Wait until the last transmission is finished
                    busy_wait!(self.i2c, tx_e);

                    // STOP
                    self.i2c.cr1.modify(|_, w| {
                        w.stop().set_bit()
                            .ack().clear_bit()
                    });

                    self.wait_busy();
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
