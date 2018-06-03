//! Serial Peripheral Interface (SPI) bus

use core::ptr;

use hal::spi::{FullDuplex, Mode, Phase, Polarity};
use nb;
use stm32f429::{SPI1, SPI2, SPI3, SPI5/*, SPI4, SPI6*/};

use gpio::gpioa::{PA5, PA6, PA7};
use gpio::gpiob::{PB13, PB14, PB15, PB5};
use gpio::gpioc::{PC2, PC10, PC11, PC12};
use gpio::gpiod::{PD3};
use gpio::gpiof::{PF7, PF8, PF9};
use gpio::{AF5, AF6};
use rcc::{APB1, APB2, Clocks};
use time::Hertz;
use dma::{DmaChannel, DmaStreamTransfer, Transfer,
          C0, C3, dma1, dma2};

/// SPI error
#[derive(Debug)]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Mode fault occurred
    ModeFault,
    /// CRC error
    Crc,
    #[doc(hidden)] _Extensible,
}

// FIXME these should be "closed" traits
/// SCK pin -- DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait SckPin<SPI> {}

/// MISO pin -- DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait MisoPin<SPI> {}

/// MOSI pin -- DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait MosiPin<SPI> {}

unsafe impl SckPin<SPI1> for PA5<AF5> {}
// unsafe impl SckPin<SPI1> for PB3<AF5> {}

unsafe impl SckPin<SPI2> for PB13<AF5> {}
unsafe impl SckPin<SPI2> for PD3<AF5> {}

// unsafe impl SckPin<SPI3> for PB3<AF6> {}
unsafe impl SckPin<SPI3> for PC10<AF6> {}

unsafe impl SckPin<SPI5> for PF7<AF5> {}

unsafe impl MisoPin<SPI1> for PA6<AF5> {}
// unsafe impl MisoPin<SPI1> for PB4<AF5> {}

unsafe impl MisoPin<SPI2> for PB14<AF5> {}
unsafe impl MisoPin<SPI2> for PC2<AF5> {}

// unsafe impl MisoPin<SPI3> for PB4<AF6> {}
unsafe impl MisoPin<SPI3> for PC11<AF6> {}

unsafe impl MisoPin<SPI5> for PF8<AF5> {}

unsafe impl MosiPin<SPI1> for PA7<AF5> {}
unsafe impl MosiPin<SPI1> for PB5<AF5> {}

unsafe impl MosiPin<SPI2> for PB15<AF5> {}

unsafe impl MosiPin<SPI3> for PB5<AF6> {}
unsafe impl MosiPin<SPI3> for PC12<AF6> {}

unsafe impl MosiPin<SPI5> for PF9<AF5> {}

/// Rx direction
pub struct DmaRx;
/// Tx direction
pub struct DmaTx;

/// Possible DMA configuration for an SPI device
pub unsafe trait SpiDmaStream<STREAM, CHANNEL, DIRECTION> {}
unsafe impl SpiDmaStream<SPI3, C0, DmaRx> for dma1::S0 {}
unsafe impl SpiDmaStream<SPI3, C0, DmaRx> for dma1::S2 {}
unsafe impl SpiDmaStream<SPI2, C0, DmaRx> for dma1::S3 {}
unsafe impl SpiDmaStream<SPI2, C0, DmaTx> for dma1::S4 {}
unsafe impl SpiDmaStream<SPI3, C0, DmaTx> for dma1::S5 {}
unsafe impl SpiDmaStream<SPI3, C0, DmaTx> for dma1::S7 {}
unsafe impl SpiDmaStream<SPI1, C3, DmaRx> for dma2::S0 {}
unsafe impl SpiDmaStream<SPI1, C3, DmaRx> for dma2::S2 {}
unsafe impl SpiDmaStream<SPI1, C3, DmaTx> for dma2::S3 {}
unsafe impl SpiDmaStream<SPI1, C3, DmaTx> for dma2::S5 {}

/// SPI peripheral operating in full duplex master mode
pub struct Spi<SPI, PINS> {
    spi: SPI,
    pins: PINS,
}

macro_rules! hal {
    ($($SPIX:ident: ($spiX:ident, $APBX:ident, $spiXen:ident, $spiXrst:ident, $pclkX:ident),)+) => {
        $(
            impl<SCK, MISO, MOSI> Spi<$SPIX, (SCK, MISO, MOSI)> {
                /// Configures the SPI peripheral to operate in full duplex master mode
                pub fn $spiX<F>(
                    spi: $SPIX,
                    pins: (SCK, MISO, MOSI),
                    mode: Mode,
                    freq: F,
                    clocks: Clocks,
                    apb: &mut $APBX,
                ) -> Self
                where
                    F: Into<Hertz>,
                    SCK: SckPin<$SPIX>,
                    MISO: MisoPin<$SPIX>,
                    MOSI: MosiPin<$SPIX>,
                {
                    // enable or reset $SPIX
                    apb.enr().modify(|_, w| w.$spiXen().set_bit());
                    apb.rstr().modify(|_, w| w.$spiXrst().set_bit());
                    apb.rstr().modify(|_, w| w.$spiXrst().clear_bit());

                    spi.cr2.write(|w| w
                                  // Tx buffer empty interrupt disable
                                  .txeie().clear_bit()
                                  // SS output enable
                                  .ssoe().set_bit()
                    );

                    let br = match clocks.$pclkX().0 / freq.into().0 {
                        0 => unreachable!(),
                        1...2 => 0b000,
                        3...5 => 0b001,
                        6...11 => 0b010,
                        12...23 => 0b011,
                        24...39 => 0b100,
                        40...95 => 0b101,
                        96...191 => 0b110,
                        _ => 0b111,
                    };

                    spi.cr1.write(|w| unsafe {
                        w
                            // 8-bit data frame format
                            .dff().clear_bit()
                            // Clock phase
                            .cpha().bit(mode.phase == Phase::CaptureOnSecondTransition)
                            // Clock polariy
                            .cpol().bit(mode.polarity == Polarity::IdleHigh)
                            // Master mode
                            .mstr().set_bit()
                            // 1 MHz
                            .br().bits(br)
                            // Enable SPI
                            .spe().set_bit()
                            // MSB transmitted first
                            .lsbfirst().clear_bit()
                            // Set NSS high
                            .ssi().set_bit()
                            // Software slave management
                            .ssm().set_bit()
                            // Disable CRC calculation
                            .crcen().clear_bit()
                            // 2-line unidirectional data mode
                            .bidimode().clear_bit()
                            // Full duplex
                            .rxonly().clear_bit()
                    });

                    Spi { spi, pins }
                }

                /// Enable transmit interrupt
                pub fn enable_send_interrupt(&mut self) {
                    self.spi.cr2.modify(|_, w| w.txeie().set_bit());
                }

                /// Disable transmit interrupt
                pub fn disable_send_interrupt(&mut self) {
                    self.spi.cr2.modify(|_, w| w.txeie().clear_bit());
                }

                /// Releases the SPI peripheral and associated pins
                pub fn free(self) -> ($SPIX, (SCK, MISO, MOSI)) {
                    (self.spi, self.pins)
                }

                /// Start a one-shot DMA transfer
                pub fn dma_write<S, T, STREAM, CHANNEL, X>(&mut self, dma: STREAM, data: S) -> X
                where
                    S: AsRef<[T]>,
                    STREAM: DmaStreamTransfer<S, T, X> + SpiDmaStream<$SPIX, CHANNEL, DmaTx>,
                    CHANNEL: DmaChannel,
                    X: Transfer<STREAM>,
                {
                    self.spi.cr2.modify(|_, w| w.txdmaen().set_bit());
                    
                    let dr: &mut T = unsafe {
                        &mut *(&self.spi.dr as *const _ as *mut T)
                    };
                    dma.start_transfer::<CHANNEL>(data, dr)
                }
            }

            impl<PINS> FullDuplex<u8> for Spi<$SPIX, PINS> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    let sr = self.spi.sr.read();

                    Err(if sr.ovr().bit_is_set() {
                        nb::Error::Other(Error::Overrun)
                    } else if sr.modf().bit_is_set() {
                        nb::Error::Other(Error::ModeFault)
                    } else if sr.crcerr().bit_is_set() {
                        nb::Error::Other(Error::Crc)
                    } else if sr.rxne().bit_is_set() {
                        // NOTE(read_volatile) read only 1 byte (the svd2rust API only allows
                        // reading a half-word)
                        return Ok(unsafe {
                            ptr::read_volatile(&self.spi.dr as *const _ as *const u8)
                        });
                    } else {
                        nb::Error::WouldBlock
                    })
                }

                fn send(&mut self, byte: u8) -> nb::Result<(), Error> {
                    let sr = self.spi.sr.read();

                    Err(/*if sr.ovr().bit_is_set() {
                        // Clear the flag:
                        unsafe {
                            ptr::read_volatile(&self.spi.dr as *const _ as *const u8)
                        };
                        nb::Error::Other(Error::Overrun)
                    } else*/ if sr.modf().bit_is_set() {
                        nb::Error::Other(Error::ModeFault)
                    } else if sr.crcerr().bit_is_set() {
                        nb::Error::Other(Error::Crc)
                    } else if sr.txe().bit_is_set() {
                        // NOTE(write_volatile) see note above
                        unsafe { ptr::write_volatile(&self.spi.dr as *const _ as *mut u8, byte) }
                        return Ok(());
                    } else {
                        nb::Error::WouldBlock
                    })
                }
            }

            impl<PINS> ::hal::blocking::spi::transfer::Default<u8> for Spi<$SPIX, PINS> {}
            // impl<PINS> ::hal::blocking::spi::write::Default<u8> for Spi<$SPIX, PINS> {}

            impl<PINS> ::hal::blocking::spi::Write<u8> for Spi<$SPIX, PINS> {
                type Error = Error;

                fn write(&mut self, bytes: &[u8]) -> Result<(), Error> {
                    for byte in bytes {
                        'l: loop {
                            let sr = self.spi.sr.read();

                            // ignore overruns because we don't care about the incoming data
                            // if sr.ovr().bit_is_set() {
                            // Err(nb::Error::Other(Error::Overrun))
                            // } else
                            if sr.modf().bit_is_set() {
                                return Err(Error::ModeFault);
                            } else if sr.crcerr().bit_is_set() {
                                return Err(Error::Crc);
                            } else if sr.txe().bit_is_set() {
                                // NOTE(write_volatile) see note above
                                unsafe { ptr::write_volatile(&self.spi.dr as *const _ as *mut u8, *byte) }
                                break 'l;
                            } else {
                                // try again
                            }
                        }
                    }

                    // wait until the transmission of the last byte is done
                    while self.spi.sr.read().bsy().bit_is_set() {}

                    // clear OVR flag
                    unsafe {
                        ptr::read_volatile(&self.spi.dr as *const _ as *const u8);
                    }
                    self.spi.sr.read();

                    Ok(())
                }
            }
        )+
    }
}

hal! {
    SPI1: (spi1, APB2, spi1en, spi1rst, pclk2),
    SPI2: (spi2, APB1, spi2en, spi2rst, pclk1),
    SPI3: (spi3, APB1, spi3en, spi3rst, pclk1),
    SPI5: (spi5, APB2, spi5en, spi5rst, pclk2),
    /* Available in the datasheet but not in the .svd:
    SPI4: (spi4, APB2, spi4en, spi4rst, pclk2),
    SPI6: (spi6, APB2, spi6en, spi6rst, pclk2),
     */
}

// FIXME not working
// TODO measure if this actually faster than the default implementation
// impl ::hal::blocking::spi::Write<u8> for Spi {
//     type Error = Error;

//     fn write(&mut self, bytes: &[u8]) -> Result<(), Error> {
//         for byte in bytes {
//             'l: loop {
//                 let sr = self.spi.sr.read();

//                 // ignore overruns because we don't care about the incoming data
//                 // if sr.ovr().bit_is_set() {
//                 // Err(nb::Error::Other(Error::Overrun))
//                 // } else
//                 if sr.modf().bit_is_set() {
//                     return Err(Error::ModeFault);
//                 } else if sr.crcerr().bit_is_set() {
//                     return Err(Error::Crc);
//                 } else if sr.txe().bit_is_set() {
//                     // NOTE(write_volatile) see note above
//                     unsafe { ptr::write_volatile(&self.spi.dr as *const _ as *mut u8, *byte) }
//                     break 'l;
//                 } else {
//                     // try again
//                 }
//             }
//         }

//         // wait until the transmission of the last byte is done
//         while self.spi.sr.read().bsy().bit_is_set() {}

//         // clear OVR flag
//         unsafe {
//             ptr::read_volatile(&self.spi.dr as *const _ as *const u8);
//         }
//         self.spi.sr.read();

//         Ok(())
//     }
// }
