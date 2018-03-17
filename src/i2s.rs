//! I2S bus
use core::marker::PhantomData;

use stm32f429::{SPI1, SPI2, SPI3};

use gpio::gpioa::PA4;
use gpio::gpiob::{PB5, PB9, PB10, PB12, PB13, PB14, PB15};
use gpio::gpioc::{PC2, PC3, PC6, PC7};
use gpio::{AF5, AF6};
use rcc::{APB1, APB2, Clocks};
// use time::{KiloHertz, MegaHertz};
use dma::*;

/// SD: Serial Data (mapped on the MOSI pin) to transmit or receive
/// the two time- multiplexed data channels (in half-duplex mode
/// only).
pub unsafe trait SdPin<I2S> {}
// unsafe impl SdPin<I2S1> for PA7<AF5> {}
// unsafe impl SdPin<I2S1> for PB5<AF5> {}
unsafe impl SdPin<SPI3> for PB5<AF6> {}
unsafe impl SdPin<SPI2> for PB15<AF5> {}
unsafe impl SdPin<SPI2> for PC3<AF5> {}

/// WS: Word Select (mapped on the NSS pin) is the data control signal output in master
/// mode and input in slave mode.
pub unsafe trait WsPin<SPI1> {}
// unsafe impl WsPin<SPI1> for PA4<AF5> {}
unsafe impl WsPin<SPI3> for PA4<AF6> {}
// unsafe impl WsPin<SPI1> for PA15<AF5> {}
// unsafe impl WsPin<SPI3> for PA15<AF6> {}
unsafe impl WsPin<SPI2> for PB9<AF5> {}
unsafe impl WsPin<SPI2> for PB12<AF5> {}

/// CK: Serial Clock (mapped on the SCK pin) is the serial clock output in master mode
/// and serial clock input in slave mode.
pub unsafe trait CkPin<I2S> {}
// unsafe impl CkPin<SPI1> for PA5<AF5> {}
// unsafe impl CkPin<SPI1> for PB3<AF5> {}
// unsafe impl CkPin<SPI3> for PB3<AF6> {}
unsafe impl CkPin<SPI2> for PB10<AF5> {}
unsafe impl CkPin<SPI2> for PB13<AF5> {}

/// SPI2ext_SD and SPI3ext_SD: additional pins (mapped on the MISO pin) to control the
/// I 2 S full duplex mode.
pub unsafe trait ExtSdPin<I2S> {}
// unsafe impl ExtSdPin<SPI1> for PA6<AF5> {}
// unsafe impl ExtSdPin<SPI1> for PB4<AF5> {}
// unsafe impl ExtSdPin<SPI3> for PB4<AF6> {}
unsafe impl ExtSdPin<SPI2> for PB14<AF6> {}
unsafe impl ExtSdPin<SPI2> for PC2<AF6> {}

/// MCK: Master Clock (mapped separately) is used, when the I 2 S is configured in master
/// mode (and when the MCKOE bit in the SPI_I2SPR register is set), to output this
/// additional clock generated at a preconfigured frequency rate equal to 256 × F S , where
/// F S is the audio sampling frequency.
pub unsafe trait MckPin<I2S> {}
unsafe impl MckPin<SPI2> for PC6<AF5> {}
unsafe impl MckPin<SPI3> for PC7<AF6> {}
// TODO: continue after PC7

/// Rx direction
pub struct DmaRx;
/// Tx direction
pub struct DmaTx;

/// Possible DMA configuration for an SPI device
pub unsafe trait I2sDmaStream<STREAM, CHANNEL, DIRECTION> {}
// DMA: there are only impls for SPI1..3, not 4..6 yet
unsafe impl I2sDmaStream<SPI3, C0, DmaRx> for dma1::S0 {}
unsafe impl I2sDmaStream<SPI3, C0, DmaRx> for dma1::S2 {}
unsafe impl I2sDmaStream<SPI2, C0, DmaRx> for dma1::S3 {}
unsafe impl I2sDmaStream<SPI2, C0, DmaTx> for dma1::S4 {}
unsafe impl I2sDmaStream<SPI3, C0, DmaTx> for dma1::S5 {}
unsafe impl I2sDmaStream<SPI3, C0, DmaTx> for dma1::S7 {}
unsafe impl I2sDmaStream<SPI1, C3, DmaRx> for dma2::S0 {}
unsafe impl I2sDmaStream<SPI1, C3, DmaRx> for dma2::S2 {}
unsafe impl I2sDmaStream<SPI1, C3, DmaTx> for dma2::S3 {}
unsafe impl I2sDmaStream<SPI1, C3, DmaTx> for dma2::S5 {}


/// Slave role (doesn't provide clock)
pub struct SlaveRole {}
/// Master role (provides clock)
pub struct MasterRole {}

/// I2S standard
pub enum I2sStandard {
    /// I2S Philips standard.
    Philips = 0b00,
    /// MSB justified standard (left justified)
    MsbJustified = 0b01,
    /// LSB justified standard (right justified)
    LsbJustified = 0b10,
    /// PCM standard
    Pcm = 0b11,
}

/// I2S peripheral
#[allow(unused)]
pub struct I2s<SPI, SD, CK, WS> {
    spi: SPI,
    sd: SD,
    ck: CK,
    ws: WS,
}

/// I2S peripheral
#[allow(unused)]
pub struct I2sOutput<Role, Data, SPI, SD, CK, WS> {
    role: PhantomData<Role>,
    data: PhantomData<Data>,
    spi: SPI,
    sd: SD,
    ck: CK,
    ws: WS,
}

pub trait I2sData {
    fn datlen() -> u8;
    fn for_u16<F: Fn(u16)>(&self, f: F);
}

impl I2sData for u16 {
    fn datlen() -> u8 {
        0b00
    }
    fn for_u16<F: Fn(u16)>(&self, f: F) {
        f(*self);
    }
}

impl I2sData for u32 {
    fn datlen() -> u8 {
        0b10
    }
    fn for_u16<F: Fn(u16)>(&self, f: F) {
        f((*self >> 16) as u16);
        f(*self as u16);
    }
}

macro_rules! hal {
    ($($SPIX:ident: ($spiX:ident, $APBX:ident, $spiXen:ident, $spiXrst:ident),)+) => {
        $(
            /// I2S interface on SPI pins
            impl<SD, CK, WS> I2s<$SPIX, SD, CK, WS> {
                /// Initialize I2S interface
                pub fn $spiX(
                    spi: $SPIX,
                    sd: SD,
                    ck: CK,
                    ws: WS,
                    _clocks: Clocks,
                    apb: &mut $APBX,
                ) -> Self where
                    SD: SdPin<$SPIX>,
                    CK: CkPin<$SPIX>,
                    WS: WsPin<$SPIX>,
                {
                    // Enable peripheral
                    apb.enr().modify(|_, w| w.$spiXen().set_bit());
                    // Reset peripheral
                    apb.rstr().modify(|_, w| w.$spiXrst().set_bit());
                    apb.rstr().modify(|_, w| w.$spiXrst().clear_bit());
                    
                    I2s { spi, sd, ck, ws }
                }

                /// Configure in slave mode as output
                pub fn into_slave_output<Data: I2sData>(self, standard: I2sStandard) -> I2sOutput<SlaveRole, Data, $SPIX, SD, CK, WS> {
                    self.spi.i2scfgr.modify(|_, w| {
                        unsafe {
                            // Select I2S mode
                            w.i2smod().set_bit()
                                // Configuration (slave, output)
                                .i2scfg().bits(0b00)
                                .i2sstd().bits(standard as u8)
                                // data length
                                .datlen().bits(Data::datlen())
                                // "auto"
                                .chlen().clear_bit()
                        }
                    });
                    // If needed, select all the potential interrupt
                    // sources and the DMA capabilities by writing the
                    // SPI_CR2 register.

                    // The I2SE bit in SPI_I2SCFGR register must be
                    // set.
                    self.spi.i2scfgr.modify(|_, w| w.i2se().set_bit());

                    I2sOutput {
                        role: PhantomData,
                        data: PhantomData,
                        spi: self.spi,
                        sd: self.sd,
                        ck: self.ck,
                        ws: self.ws,
                    }
                }
            }

            impl<Role, Data: I2sData, SD, CK, WS> I2sOutput<Role, Data, $SPIX, SD, CK, WS> {
                /// Disable and return `I2s`
                pub fn into_i2s(self) -> I2s<$SPIX, SD, CK, WS> {
                    // Wait
                    while self.spi.sr.read().bsy().bit() ||
                        ! self.spi.sr.read().txe().bit() {}
                    // Disable first
                    self.spi.i2scfgr.modify(|_, w| w.i2se().clear_bit());

                    I2s {
                        spi: self.spi,
                        sd: self.sd,
                        ck: self.ck,
                        ws: self.ws,
                    }
                }

                /// Write data word
                pub fn write(&mut self, data: Data) {
                    data.for_u16(|word| {
                        while ! self.spi.sr.read().txe().bit() {}
                        self.spi.dr.write(|w| unsafe { w.dr().bits(word) });
                    });
                }

                pub fn dma_write<'a, S, C>(&mut self, data: &'a [Data], stream: S) -> Result<S, S>
                where S: DmaStream + I2sDmaStream<$SPIX, C, DmaTx>,
                      C: DmaChannel,
                {
                    // Let SPI/I2S make a DMA request whenever the TXE flag is set
                    self.spi.cr2.modify(|_, w| w.txdmaen().set_bit());

                    let transfer: Transfer<'a, S, C> = stream.memory_to_peripheral(data, &self.spi.dr);
                    transfer.wait()
                }
            }
        )+
    }
}

hal! {
    SPI1: (spi1, APB2, spi1en, spi1rst),
    SPI2: (spi2, APB1, spi2en, spi2rst),
    SPI3: (spi3, APB1, spi3en, spi3rst),
    // SPI4: (spi4, APB2, spi4en, spi4rst),
    // SPI5: (spi5, APB2, spi5en, spi5rst),
    // SPI6: (spi6, APB2, spi6en, spi6rst),
}
