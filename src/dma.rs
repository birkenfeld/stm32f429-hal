//! DMA abstractions

use core::mem::size_of;
use core::ops::Not;
use rcc::AHB1;

/// DMA channel, implemented by the types `C0`, `C1`, `C2`, …
pub trait DmaChannel {
    /// Numeric channel number
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

/// Split the DMA device into separate streams.
pub trait DmaExt {
    /// Target type
    type Streams;

    /// Split into separate streams.
    fn split(self, ahb: &mut AHB1) -> Self::Streams;
}

/// Events to enable interrupts for.
pub enum Event {
    /// Half transfer
    HalfTransfer,
    /// Transfer complete
    TransferComplete,
}

#[derive(Debug, Clone, Copy)]
enum DoubleBuffer {
    Memory0 = 0,
    Memory1 = 1,
}

impl Not for DoubleBuffer {
    type Output = Self;
    fn not(self) -> Self::Output {
        match self {
            DoubleBuffer::Memory0 =>
                DoubleBuffer::Memory1,
            DoubleBuffer::Memory1 =>
                DoubleBuffer::Memory0,
        }
    }
}

/// DMA stream peripheral
pub trait DmaStream {
    /// Enable interrupt
    fn listen(&mut self, event: Event);
    /// Disable interrupt
    fn unlisten(&mut self, event: Event);

    /// Transfer is complete?
    fn is_complete(&self) -> bool;
    /// Transfer has error?
    fn has_error(&self) -> bool;
    /// Reset after a transfer
    fn reset(&mut self);
}

/// DMA stream that can start DMA transfer `X`
pub trait DmaStreamTransfer<S, T, X: Transfer<Self>>: DmaStream + Sized {
    /// Start DMA transfer
    fn start_transfer<CHANNEL: DmaChannel>(self, source: S, target: &mut T) -> X;
}

/// DMA transfer
pub trait Transfer<STREAM>: Sized {
    /// Transfer is complete?
    fn is_complete(&self) -> bool;
    /// Transfer has error?
    fn has_error(&self) -> bool;
    /// Reset after a transfer
    ///
    /// Consumes the finished transfer and returns the stream.
    fn reset(self) -> STREAM;

    /// Wait until transfer is either complete or has error.
    fn wait(self) -> Result<STREAM, STREAM> {
        while !self.is_complete() && !self.has_error() {}
        if self.is_complete() {
            Ok(self.reset())
        } else {
            Err(self.reset())
        }
    }
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
            /// Peripheral abstraction for DMA
            pub mod $dmaX {
                use stm32f429::{$DMAX, dma2};

                use rcc::AHB1;
                use dma::{DmaExt, DmaStream, DmaStreamTransfer, DmaChannel,
                          Event, data_size};

                /// The numbered DMA streams of a device that you can
                /// use separately.
                #[derive(Debug)]
                pub struct Streams {
                    $(
                        /// DMA stream `$sx`
                        pub $sx: $SX
                    ),+
                }

                $(
                    /// A handle to the `$SX` DMA peripheral
                    #[derive(Debug)]
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
                    
                    impl<'s, S> DmaStreamTransfer<(&'s [S], &'s [S]), S, $sx::DoubleBufferedTransfer<S>> for $SX {
                        /// Configure, enable, and return a double-buffered DMA transfer.
                        fn start_transfer<CHANNEL: DmaChannel>(mut self, (source0, source1): (&'s [S], &'s [S]), target: &mut S) -> $sx::DoubleBufferedTransfer<S> {
                            assert_eq!(source0.len(), source1.len());

                            self.cr().modify(|_, w| unsafe {
                                w.msize().bits(data_size::<S>())
                                    .minc().set_bit()
                                    .psize().bits(data_size::<S>())
                                    .pinc().clear_bit()
                                    .dbm().set_bit()
                                    .ct().clear_bit()
                                    .circ().set_bit()
                                // Memory to peripheral
                                    .dir().bits(0b01)
                                    .chsel().bits(CHANNEL::channel())
                            });

                            let source0_addr = &source0[0] as *const _ as u32;
                            self.m0ar().write(|w| unsafe { w.bits(source0_addr) });
                            let source1_addr = &source1[0] as *const _ as u32;
                            self.m1ar().write(|w| unsafe { w.bits(source1_addr) });
                            let source_len = source0.len() as u32;
                            self.ndtr().write(|w| unsafe { w.bits(source_len) });
                            let target_addr = target as *const _ as u32;
                            self.par().write(|w| unsafe { w.bits(target_addr) });

                            // Enable Stream
                            self.cr().modify(|_, w| w.en().set_bit());

                            $sx::DoubleBufferedTransfer::new(self)
                        }
                    }

                    impl<T, S: AsRef<[T]>> DmaStreamTransfer<S, T, $sx::OneShotTransfer<S>> for $SX {
                        /// Configure, enable, and return a double-buffered DMA transfer.
                        fn start_transfer<CHANNEL: DmaChannel>(mut self, source: S, target: &mut T) -> $sx::OneShotTransfer<S> {
                            self.cr().modify(|_, w| unsafe {
                                w.msize().bits(data_size::<T>())
                                    .minc().set_bit()
                                    .psize().bits(data_size::<T>())
                                    .pinc().clear_bit()
                                    .dbm().clear_bit()
                                    .ct().clear_bit()
                                    .circ().clear_bit()
                                    // Memory to peripheral
                                    .dir().bits(0b01)
                                    .chsel().bits(CHANNEL::channel())
                            });

                            let source_addr = source.as_ref() as *const _ as *const () as u32;
                            self.m0ar().write(|w| unsafe { w.bits(source_addr) });
                            let source_len = source.as_ref().len() as u32;
                            self.ndtr().write(|w| unsafe { w.bits(source_len) });
                            let target_addr = target as *const _ as u32;
                            self.par().write(|w| unsafe { w.bits(target_addr) });

                            // Enable Stream
                            self.cr().modify(|_, w| w.en().set_bit());

                            $sx::OneShotTransfer::new(self, source)
                        }
                    }

                    /// Contains the `DoubleBufferedTransfer` and the `OneShotTransfer` for `$SX`
                    pub mod $sx {
                        use core::marker::PhantomData;
                        use dma::{DmaStream, Transfer, DoubleBuffer};
                        use super::$SX;

                        /// Double-buffered DMA transfer
                        pub struct DoubleBufferedTransfer<S> {
                            /// So that `poll()` can detect a buffer switch
                            sent: [bool; 2],
                            _source_el: PhantomData<S>,
                            stream: $SX,
                        }

                        impl<S> Transfer<$SX> for DoubleBufferedTransfer<S> {
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

                        impl<S> DoubleBufferedTransfer<S> {
                            /// Construct a new DMA transfer state,
                            /// returned by `start_transfer` which
                            /// configures and enables the stream
                            /// before.
                            pub fn new<'s>(stream: $SX) -> Self {
                                Self {
                                    sent: [false; 2],
                                    _source_el: PhantomData,
                                    stream,
                                }
                            }

                            /// Return the index of the buffer currently being sent
                            #[inline]
                            fn front_buffer(&mut self) -> DoubleBuffer {
                                if self.stream.cr().read().ct().bit() {
                                    DoubleBuffer::Memory1
                                } else {
                                    DoubleBuffer::Memory0
                                }
                            }

                            /// Return the index of the buffer **not** currently being sent
                            #[inline]
                            fn back_buffer(&mut self) -> DoubleBuffer {
                                ! self.front_buffer()
                            }

                            /// Has the back buffer been sent?
                            ///
                            /// As this is used for polling, the
                            /// function updates the `sent` status of
                            /// the front buffer.
                            pub fn writable(&mut self) -> bool {
                                // Mark front buffer as being sent
                                self.sent[self.front_buffer() as usize] = true;

                                self.sent[self.back_buffer() as usize]
                            }

                            /// Update the back buffer.
                            pub fn write<'s>(&mut self, source: &'s [S]) -> Result<(), ()> {
                                if self.has_error() {
                                    return Err(())
                                }

                                let source_addr = &source[0] as *const _ as u32;
                                let bb = self.back_buffer();
                                match bb {
                                    DoubleBuffer::Memory0 =>
                                        self.stream.m0ar().write(|w| unsafe { w.bits(source_addr) }),
                                    DoubleBuffer::Memory1 =>
                                        self.stream.m1ar().write(|w| unsafe { w.bits(source_addr) }),
                                }
                                // Let `writable()` mark it when it becomes the `front_buffer()`
                                self.sent[bb as usize] = false;

                                Ok(())
                            }
                        }

                        /// One-shot DMA transfer
                        pub struct OneShotTransfer<S> {
                            source: S,
                            stream: $SX,
                        }

                        impl<S> Transfer<$SX> for OneShotTransfer<S> {
                            fn is_complete(&self) -> bool {
                                self.stream.is_complete()
                            }

                            fn has_error(&self) -> bool {
                                self.stream.has_error()
                            }

                            fn reset(mut self) -> $SX {
                                drop(self.source);
                                self.stream.reset();
                                self.stream
                            }
                        }

                        impl<S> OneShotTransfer<S> {
                            /// Construct a new DMA transfer state,
                            /// returned by `start_transfer` which
                            /// configures and enables the stream
                            /// before.
                            pub fn new<'s>(stream: $SX, source: S) -> Self {
                                Self {
                                    source,
                                    stream,
                                }
                            }

                            /// debug
                            pub fn status(&mut self) -> u32 {
                                self.stream.cr().read().bits()
                            }
                        }
                    }
                )+

                impl DmaExt for $DMAX {
                    type Streams = Streams;

                    fn split(self, ahb: &mut AHB1) -> Streams {
                        ahb.enr().modify(|_, w| w.$dmaXen().set_bit());
                        ahb.rstr().modify(|_, w| w.$dmaXrst().set_bit());
                        ahb.rstr().modify(|_, w| w.$dmaXrst().clear_bit());

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
