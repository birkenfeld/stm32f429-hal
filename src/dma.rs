//! DMA abstractions

use core::mem::size_of;
use rcc::AHB1;

pub trait DmaChannel {
    fn channel() -> u8;
}

/// DMA channel
pub struct C0;
impl DmaChannel for C0 {
    fn channel() -> u8 { 0 }
}
/// DMA channel
pub struct C1;
impl DmaChannel for C1 {
    fn channel() -> u8 { 1 }
}
/// DMA channel
pub struct C2;
impl DmaChannel for C2 {
    fn channel() -> u8 { 2 }
}
/// DMA channel
pub struct C3;
impl DmaChannel for C3 {
    fn channel() -> u8 { 3 }
}
/// DMA channel
pub struct C4;
impl DmaChannel for C4 {
    fn channel() -> u8 { 4 }
}
/// DMA channel
pub struct C5;
impl DmaChannel for C5 {
    fn channel() -> u8 { 5 }
}
/// DMA channel
pub struct C6;
impl DmaChannel for C6 {
    fn channel() -> u8 { 6 }
}
/// DMA channel
pub struct C7;
impl DmaChannel for C7 {
    fn channel() -> u8 { 7 }
}


pub trait DmaExt {
    type Streams;

    fn split(self, ahb: &mut AHB1) -> Self::Streams;
}

pub enum Event {
    HalfTransfer,
    TransferComplete,
}

pub trait Transfer<STREAM, S, T>: Sized {
    fn new(stream: STREAM, source: S, target: &mut T) -> Result<Self, STREAM>;

    fn is_complete(&self) -> bool;
    fn has_error(&self) -> bool;
    fn reset(self) -> STREAM;

    fn wait(self) -> Result<STREAM, STREAM> {
        while !self.is_complete() && !self.has_error() {}
        if self.is_complete() {
            Ok(self.reset())
        } else {
            Err(self.reset())
        }
    }
}


pub trait DmaStream: Sized {
    fn listen(&mut self, event: Event);
    fn unlisten(&mut self, event: Event);
    fn transfer<S, T, X: Transfer<Self, S, T>>(self, source: S, target: &mut T) -> Result<X, Self>;
    fn is_complete(&self) -> bool;
    fn has_error(&self) -> bool;
    fn reset(&mut self);
}

macro_rules! dma {
    ($($DMAX:ident: ($dmaX:ident, $dmaXen:ident, $dmaXrst:ident, {
        $($SX:ident: (
            $sx:ident,
            $crX:ident: $CRX:ident,
            $ndtrX:ident: $NDTRX:ident,
            $parX:ident: $PARX:ident,
            $m0arX:ident: $M0ARX:ident,
            $m1arX:ident: $M1ARX:ident,
            $isr:ident: $ISR:ident,
            $ifcr:ident: $IFCR:ident,
            $tcif:ident, $teif:ident,
            $ctcif:ident, $cteif:ident,
        ),)+
    }),)+) => {
        $(
            pub mod $dmaX {
                use stm32f429::{$DMAX, dma2};

                use rcc::AHB1;
                use dma::{DmaExt, DmaStream,
                          Transfer, Event};

                pub struct Streams {
                    $(pub $sx: $SX),+
                }

                $(
                    pub struct $SX { _0: () }

                    impl $SX {
                        fn isr(&self) -> dma2::$isr::R {
                            // NOTE(unsafe) atomic read with no side effects
                            unsafe { (*$DMAX::ptr()).$isr.read() }
                        }

                        fn ifcr(&self) -> &dma2::$IFCR {
                            unsafe { &(*$DMAX::ptr()).$ifcr }
                        }

                        fn cr(&mut self) -> &dma2::$CRX {
                            unsafe { &(*$DMAX::ptr()).$crX }
                        }

                        fn ndtr(&mut self) -> &dma2::$NDTRX {
                            unsafe { &(*$DMAX::ptr()).$ndtrX }
                        }

                        // fn get_ndtr(&self) -> u32 {
                        //     // NOTE(unsafe) atomic read with no side effects
                        //     unsafe { (*$DMAX::ptr()).$ndtrX.read().bits() }
                        // }

                        fn par(&mut self) -> &dma2::$PARX {
                            unsafe { &(*$DMAX::ptr()).$parX }
                        }

                        fn m0ar(&mut self) -> &dma2::$M0ARX {
                            unsafe { &(*$DMAX::ptr()).$m0arX }
                        }

                        fn m1ar(&mut self) -> &dma2::$M1ARX {
                            unsafe { &(*$DMAX::ptr()).$m1arX }
                        }
                    }

                    impl DmaStream for $SX {
                        fn listen(&mut self, event: Event) {
                            match event {
                                Event::HalfTransfer => self.cr().modify(|_, w| w.htie().set_bit()),
                                Event::TransferComplete => {
                                    self.cr().modify(|_, w| w.tcie().set_bit())
                                }
                            }
                        }

                        fn unlisten(&mut self, event: Event) {
                            match event {
                                Event::HalfTransfer => {
                                    self.cr().modify(|_, w| w.htie().clear_bit())
                                },
                                Event::TransferComplete => {
                                    self.cr().modify(|_, w| w.tcie().clear_bit())
                                }
                            }
                        }

                        fn transfer<S, T, X: Transfer<Self, S, T>>(self, source: S, target: &mut T) -> Result<X, Self> {
                            X::new(self, source, target)
                        }

                        fn is_complete(&self) -> bool {
                            self.isr().$tcif().bit()
                        }

                        fn has_error(&self) -> bool {
                            self.isr().$teif().bit()
                        }

                        fn reset(&mut self) {
                            // Disable Stream
                            self.cr().modify(|_, w| w.en().clear_bit());
                            
                            // Clear status bits
                            self.ifcr().modify(|_, w| {
                                w.$ctcif().set_bit()
                                    .$cteif().set_bit()
                            });
                        }
                    }

                    pub mod $sx {
                        use core::marker::PhantomData;
                        use dma::{DmaStream, DmaChannel,
                                  Transfer, data_size};
                        use super::$SX;

                        pub struct MemoryToPeripheral<'s, S: 's + AsRef<[SE]>, SE, T, CHANNEL> {
                            _source: PhantomData<&'s S>,
                            _source_element: PhantomData<SE>,
                            _target: PhantomData<T>,
                            stream: $SX,
                            _channel: PhantomData<CHANNEL>,
                        }

                        impl<'s, SE, S: 's + AsRef<[SE]>, T, CHANNEL: DmaChannel> Transfer<$SX, &'s S, T> for MemoryToPeripheral<'s, S, SE, T, CHANNEL> {
                            fn new(mut stream: $SX, source: &'s S, target: &mut T) -> Result<Self, $SX> {
                                stream.cr().modify(|_, w| unsafe {
                                    w.msize().bits(data_size::<SE>())
                                        .minc().set_bit()
                                        .psize().bits(data_size::<T>())
                                        .pinc().clear_bit()
                                        .circ().clear_bit()
                                        // Memory to peripheral
                                        .dir().bits(0b01)
                                        .chsel().bits(CHANNEL::channel())
                                });

                                let source_addr = &source.as_ref() as *const _ as u32;
                                stream.m0ar().write(|w| unsafe { w.bits(source_addr) });
                                let source_len = source.as_ref().len() as u32;
                                stream.ndtr().write(|w| unsafe { w.bits(source_len) });
                                let target_addr = target as *const _ as u32;
                                stream.par().write(|w| unsafe { w.bits(target_addr) });

                                // Enable Stream
                                stream.cr().modify(|_, w| w.en().set_bit());

                                Ok(Self {
                                    _source: PhantomData,
                                    _source_element: PhantomData,
                                    _target: PhantomData,
                                    stream,
                                    _channel: PhantomData,
                                })
                            }

                            fn is_complete(&self) -> bool {
                                self.stream.is_complete()
                            }

                            fn has_error(&self) -> bool {
                                self.stream.has_error()
                            }

                            fn reset(mut self) -> $SX {
                                self.stream.reset();
                                self.stream
                            }
                        }

                        pub struct DoubleBufferedTransfer<'s, S: 's, SE, T, CHANNEL> {
                            /// So that `poll()` can detect a buffer switch
                            last_ct: usize,
                            source0: &'s S,
                            source1: &'s S,
                            _source_element: PhantomData<SE>,
                            _target: PhantomData<T>,
                            stream: $SX,
                            _channel: PhantomData<CHANNEL>,
                        }

                        impl<'s, SE, S: 's + AsRef<[SE]>, T, CHANNEL: DmaChannel> Transfer<$SX, (&'s S, &'s S), T> for DoubleBufferedTransfer<'s, S, SE, T, CHANNEL> {
                            fn new(mut stream: $SX, source: (&'s S, &'s S), target: &mut T) -> Result<Self, $SX> {
                                if source.0.as_ref().len() != source.1.as_ref().len() {
                                    return Err(stream)
                                }

                                stream.cr().modify(|_, w| unsafe {
                                    w.msize().bits(data_size::<SE>())
                                        .minc().set_bit()
                                        .psize().bits(data_size::<T>())
                                        .pinc().clear_bit()
                                        .dbm().set_bit()
                                        .ct().clear_bit()
                                        .circ().set_bit()
                                        // Memory to peripheral
                                        .dir().bits(0b01)
                                        .chsel().bits(CHANNEL::channel())
                                });

                                let source0_addr = source.0 as *const _ as u32;
                                stream.m0ar().write(|w| unsafe { w.bits(source0_addr) });
                                let source1_addr = source.1 as *const _ as u32;
                                stream.m1ar().write(|w| unsafe { w.bits(source1_addr) });
                                let source_len = source.0.as_ref().len() as u32;
                                stream.ndtr().write(|w| unsafe { w.bits(source_len) });
                                let target_addr = target as *const _ as u32;
                                stream.par().write(|w| unsafe { w.bits(target_addr) });

                                // Enable Stream
                                stream.cr().modify(|_, w| w.en().set_bit());

                                Ok(DoubleBufferedTransfer {
                                    last_ct: 0,
                                    source0: source.0,
                                    source1: source.1,
                                    _source_element: PhantomData,
                                    _target: PhantomData,
                                    _channel: PhantomData,
                                    stream,
                                })
                            }

                            fn is_complete(&self) -> bool {
                                self.stream.is_complete()
                            }

                            fn has_error(&self) -> bool {
                                self.stream.has_error()
                            }

                            fn reset(mut self) -> $SX {
                                self.stream.reset();
                                self.stream
                            }
                        }

                        impl<'s, SE, S: 's + AsRef<[SE]>, T, CHANNEL: DmaChannel> DoubleBufferedTransfer<'s, S, SE, T, CHANNEL> {
                            pub fn poll<F: FnOnce(&'s S) -> &'s S>(mut self, f: F) -> Result<Self, $SX> {
                                if self.has_error() {
                                    return Err(self.reset())
                                }

                                let ct = if self.stream.cr().read().ct().bit() {
                                    0
                                } else {
                                    1
                                };
                                if ct != self.last_ct {
                                    if ct == 0 {
                                        // Currently transfering from source0
                                        self.source1 = f(self.source1);
                                        assert_eq!(self.source1.as_ref().len(), self.source0.as_ref().len());
                                        let source_addr = self.source1 as *const _ as u32;
                                        self.stream.m1ar().write(|w| unsafe { w.bits(source_addr) });
                                    } else if ct == 1 {
                                        // Currently transfering from source1
                                        self.source0 = f(self.source0);
                                        assert_eq!(self.source0.as_ref().len(), self.source1.as_ref().len());
                                        let source_addr = self.source0 as *const _ as u32;
                                        self.stream.m0ar().write(|w| unsafe { w.bits(source_addr) });
                                    }

                                    self.last_ct = ct;
                                }

                                Ok(self)
                            }
                        }
                    }
                )+

                impl DmaExt for $DMAX {
                    type Streams = Streams;

                    fn split(self, ahb: &mut AHB1) -> Streams {
                        ahb.enr().modify(|_, w| w.$dmaXen().set_bit());

                        // reset the DMA control registers (stops all on-going transfers)
                        $(
                            self.$crX.reset();
                        )+

                            Streams {
                                $($sx: $SX { _0: () }),+
                            }
                    }
                }
            }
        )+
    }
}

dma! {
    DMA1: (dma1, dma1en, dma1rst, {
        S0: (
            s0,
            s0cr: S0CR,
            s0ndtr: S0NDTR,
            s0par: S0PAR,
            s0m0ar: S0M0AR,
            s0m1ar: S0M1AR,
            lisr: LISR,
            lifcr: LIFCR,
            tcif0, teif0,
            ctcif0, cteif0,
        ),
        S1: (
            s1,
            s1cr: S1CR,
            s1ndtr: S1NDTR,
            s1par: S1PAR,
            s1m0ar: S1M0AR,
            s1m1ar: S1M1AR,
            lisr: LISR,
            lifcr: LIFCR,
            tcif1, teif1,
            ctcif1, cteif1,
        ),
        S2: (
            s2,
            s2cr: S2CR,
            s2ndtr: S2NDTR,
            s2par: S2PAR,
            s2m0ar: S2M0AR,
            s2m1ar: S2M1AR,
            lisr: LISR,
            lifcr: LIFCR,
            tcif2, teif2,
            ctcif2, cteif2,
        ),
        S3: (
            s3,
            s3cr: S3CR,
            s3ndtr: S3NDTR,
            s3par: S3PAR,
            s3m0ar: S3M0AR,
            s3m1ar: S3M1AR,
            lisr: LISR,
            lifcr: LIFCR,
            tcif3, teif3,
            ctcif3, cteif3,
        ),
        S4: (
            s4,
            s4cr: S4CR,
            s4ndtr: S4NDTR,
            s4par: S4PAR,
            s4m0ar: S4M0AR,
            s4m1ar: S4M1AR,
            hisr: HISR,
            hifcr: HIFCR,
            tcif4, teif4,
            ctcif4, cteif4,
        ),
        S5: (
            s5,
            s5cr: S5CR,
            s5ndtr: S5NDTR,
            s5par: S5PAR,
            s5m0ar: S5M0AR,
            s5m1ar: S5M1AR,
            hisr: HISR,
            hifcr: HIFCR,
            tcif5, teif5,
            ctcif5, cteif5,
        ),
        S6: (
            s6,
            s6cr: S6CR,
            s6ndtr: S6NDTR,
            s6par: S6PAR,
            s6m0ar: S6M0AR,
            s6m1ar: S6M1AR,
            hisr: HISR,
            hifcr: HIFCR,
            tcif6, teif6,
            ctcif6, cteif6,
        ),
        S7: (
            s7,
            s7cr: S7CR,
            s7ndtr: S7NDTR,
            s7par: S7PAR,
            s7m0ar: S7M0AR,
            s7m1ar: S7M1AR,
            hisr: HISR,
            hifcr: HIFCR,
            tcif7, teif7,
            ctcif7, cteif7,
        ),
    }),
    DMA2: (dma2, dma2en, dma2rst, {
        S0: (
            s0,
            s0cr: S0CR,
            s0ndtr: S0NDTR,
            s0par: S0PAR,
            s0m0ar: S0M0AR,
            s0m1ar: S0M1AR,
            lisr: LISR,
            lifcr: LIFCR,
            tcif0, teif0,
            ctcif0, cteif0,
        ),
        S1: (
            s1,
            s1cr: S1CR,
            s1ndtr: S1NDTR,
            s1par: S1PAR,
            s1m0ar: S1M0AR,
            s1m1ar: S1M1AR,
            lisr: LISR,
            lifcr: LIFCR,
            tcif1, teif1,
            ctcif1, cteif1,
        ),
        S2: (
            s2,
            s2cr: S2CR,
            s2ndtr: S2NDTR,
            s2par: S2PAR,
            s2m0ar: S2M0AR,
            s2m1ar: S2M1AR,
            lisr: LISR,
            lifcr: LIFCR,
            tcif2, teif2,
            ctcif2, cteif2,
        ),
        S3: (
            s3,
            s3cr: S3CR,
            s3ndtr: S3NDTR,
            s3par: S3PAR,
            s3m0ar: S3M0AR,
            s3m1ar: S3M1AR,
            lisr: LISR,
            lifcr: LIFCR,
            tcif3, teif3,
            ctcif3, cteif3,
        ),
        S4: (
            s4,
            s4cr: S4CR,
            s4ndtr: S4NDTR,
            s4par: S4PAR,
            s4m0ar: S4M0AR,
            s4m1ar: S4M1AR,
            hisr: HISR,
            hifcr: HIFCR,
            tcif4, teif4,
            ctcif4, cteif4,
        ),
        S5: (
            s5,
            s5cr: S5CR,
            s5ndtr: S5NDTR,
            s5par: S5PAR,
            s5m0ar: S5M0AR,
            s5m1ar: S5M1AR,
            hisr: HISR,
            hifcr: HIFCR,
            tcif5, teif5,
            ctcif5, cteif5,
        ),
        S6: (
            s6,
            s6cr: S6CR,
            s6ndtr: S6NDTR,
            s6par: S6PAR,
            s6m0ar: S6M0AR,
            s6m1ar: S6M1AR,
            hisr: HISR,
            hifcr: HIFCR,
            tcif6, teif6,
            ctcif6, cteif6,
        ),
        S7: (
            s7,
            s7cr: S7CR,
            s7ndtr: S7NDTR,
            s7par: S7PAR,
            s7m0ar: S7M0AR,
            s7m1ar: S7M1AR,
            hisr: HISR,
            hifcr: HIFCR,
            tcif7, teif7,
            ctcif7, cteif7,
        ),
    }),
}

fn data_size<T>() -> u8 {
    match size_of::<T>() {
        1 => 0b00,
        2 => 0b01,
        4 => 0b10,
        _ => panic!("No such data size"),
    }
}
