//! Reset and Clock Control

use cast::u32;
use stm32f429::{rcc, RCC};

use flash::ACR;
use time::Hertz;

/// Extension trait that constrains the `RCC` peripheral
pub trait RccExt {
    /// Constrains the `RCC` peripheral so it plays nicely with the other abstractions
    fn constrain(self) -> Rcc;
}

impl RccExt for RCC {
    fn constrain(self) -> Rcc {
        Rcc {
            ahb1: AHB1 { _0: () },
            ahb2: AHB2 { _0: () },
            ahb3: AHB3 { _0: () },
            apb1: APB1 { _0: () },
            apb2: APB2 { _0: () },
            cfgr: CFGR {
                hclk: None,
                pclk1: None,
                pclk2: None,
                sysclk: None,
            },
        }
    }
}

/// Constrained RCC peripheral
pub struct Rcc {
    /// AMBA High-performance Bus 1 (AHB1) registers
    pub ahb1: AHB1,
    /// AMBA High-performance Bus 2 (AHB2) registers
    pub ahb2: AHB2,
    /// AMBA High-performance Bus 3 (AHB3) registers
    pub ahb3: AHB3,
    /// Advanced Peripheral Bus 1 (APB1) registers
    pub apb1: APB1,
    /// Advanced Peripheral Bus 2 (APB2) registers
    pub apb2: APB2,
    /// Clock configuration
    pub cfgr: CFGR,
}

/// AMBA High-performance Bus 1 (AHB1) registers
pub struct AHB1 {
    _0: (),
}

#[allow(unused)]
impl AHB1 {
    pub(crate) fn enr(&mut self) -> &rcc::AHB1ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb1enr }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::AHB1RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb1rstr }
    }
}

/// AMBA High-performance Bus 2 (AHB2) registers
pub struct AHB2 {
    _0: (),
}

#[allow(unused)]
impl AHB2 {
    pub(crate) fn enr(&mut self) -> &rcc::AHB2ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb2enr }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::AHB2RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb2rstr }
    }
}

/// AMBA High-performance Bus 3 (AHB3) registers
pub struct AHB3 {
    _0: (),
}

#[allow(unused)]
impl AHB3 {
    pub(crate) fn enr(&mut self) -> &rcc::AHB3ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb3enr }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::AHB3RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb3rstr }
    }
}

/// Advanced Peripheral Bus 1 (APB1) registers
pub struct APB1 {
    _0: (),
}

impl APB1 {
    pub(crate) fn enr(&mut self) -> &rcc::APB1ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb1enr }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::APB1RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb1rstr }
    }
}

/// Advanced Peripheral Bus 2 (APB2) registers
pub struct APB2 {
    _0: (),
}

#[allow(unused)]
impl APB2 {
    pub(crate) fn enr(&mut self) -> &rcc::APB2ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb2enr }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::APB2RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb2rstr }
    }
}

const HSI: u32 = 16_000_000; // 16 MHz

/// Clock configuration
#[derive(Debug)]
pub struct CFGR {
    hclk: Option<u32>,
    pclk1: Option<u32>,
    pclk2: Option<u32>,
    sysclk: Option<u32>,
}

impl CFGR {
    /// Sets a frequency for the AHB bus
    pub fn hclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.hclk = Some(freq.into().0);
        self
    }

    /// Sets a frequency for the APB1 bus
    pub fn pclk1<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.pclk1 = Some(freq.into().0);
        self
    }

    /// Sets a frequency for the APB2 bus
    pub fn pclk2<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.pclk2 = Some(freq.into().0);
        self
    }

    /// Sets the system (core) frequency
    pub fn sysclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.sysclk = Some(freq.into().0);
        self
    }

    /// Freezes the clock configuration, making it effective
    pub fn freeze(self, acr: &mut ACR) -> Clocks {
        let target_sysclk = self.sysclk.unwrap_or(HSI);
        let (pll_n_bits, sysclk) = if target_sysclk > HSI {
            (Some(32 * (target_sysclk / 1000000) / (HSI / 1000000)),
             16 * target_sysclk / 16)
        } else {
            (None, HSI)
        };

        assert!(sysclk <= 180_000_000);

        // Prescaler factor
        let hpre_bits = self.hclk
            .map(|hclk| match sysclk / hclk {
                0 => unreachable!(),
                1 => 0b0111,  // sysclk not divided
                2 => 0b1000,
                3...5 => 0b1001,
                6...11 => 0b1010,
                12...39 => 0b1011,
                40...95 => 0b1100,
                96...191 => 0b1101,
                192...383 => 0b1110,
                _ => 0b1111,
            })
            .unwrap_or(0b0111);  // sysclk not divided

        let hclk = sysclk / (1 << (hpre_bits - 0b0111));

        assert!(hclk <= 180_000_000);

        let ppre1_bits = self.pclk1
            .map(|pclk1| match hclk / pclk1 {
                0 => unreachable!(),
                1 => 0b011,
                2 => 0b100,
                3...5 => 0b101,
                6...11 => 0b110,
                _ => 0b111,
            })
            .unwrap_or(0b011);

        let ppre1 = 1 << (ppre1_bits - 0b011);
        let pclk1 = hclk / u32(ppre1);

        // Must not exceed 45 MHz!
        assert!(pclk1 <= 45_000_000);

        let ppre2_bits = self.pclk2
            .map(|pclk2| match hclk / pclk2 {
                0 => unreachable!(),
                1 => 0b011,
                2 => 0b100,
                3...5 => 0b101,
                6...11 => 0b110,
                _ => 0b111,
            })
            .unwrap_or(0b011);

        let ppre2 = 1 << (ppre2_bits - 0b011);
        let pclk2 = hclk / u32(ppre2);

        // Must not exceed 90 MHz!
        assert!(pclk2 <= 90_000_000);

        // adjust flash wait states
        acr.acr().modify(|_, w| {
            w.latency().bits(if sysclk <= 24_000_000 {
                0b000
            } else if sysclk <= 48_000_000 {
                0b001
            } else if sysclk <= 72_000_000 {
                0b010
            } else if sysclk <= 96_000_000 {
                0b011
            } else if sysclk <= 120_000_000 {
                0b100
            } else if sysclk <= 144_000_000 {
                0b101
            } else if sysclk <= 168_000_000 {
                0b110
            } else {
                0b111
            })
        });

        let rcc = unsafe { &*RCC::ptr() };
        if let Some(pll_n_bits) = pll_n_bits {
            // use PLL as source
            rcc.pllcfgr.modify(|_, w| unsafe { w.plln().bits(pll_n_bits as u16) });

            // Enable PLL
            rcc.cr.modify(|_, w| w.pllon().set_bit());
            // Wait for PLL ready
            while rcc.cr.read().pllrdy().bit_is_clear() {}

            // SW: PLL selected as system clock
            rcc.cfgr.modify(|_, w| unsafe {
                w
                    // APB high-speed prescaler (APB2)
                    .ppre2()
                    .bits(ppre2_bits)
                    // APB Low speed prescaler (APB1)
                    .ppre1()
                    .bits(ppre1_bits)
                    // AHB prescaler
                    .hpre()
                    .bits(hpre_bits)
                    // System clock switch
                    .sw()
                    // PLL selected as system clock
                    .bits(0b10)
            });
        } else {
            // use HSI as source

            // SW: HSI selected as system clock
            rcc.cfgr.write(|w| unsafe {
                w.ppre2()
                    .bits(ppre2_bits)
                    .ppre1()
                    .bits(ppre1_bits)
                    .hpre()
                    .bits(hpre_bits)
                    .sw()
                    .bits(0b00)
            });
        }

        Clocks {
            hclk: Hertz(hclk),
            pclk1: Hertz(pclk1),
            pclk2: Hertz(pclk2),
            ppre1,
            ppre2,
            sysclk: Hertz(sysclk),
        }
    }
}

/// Frozen clock frequencies
///
/// The existence of this value indicates that the clock configuration can no longer be changed
#[derive(Clone, Copy, Debug)]
pub struct Clocks {
    hclk: Hertz,
    pclk1: Hertz,
    pclk2: Hertz,
    ppre1: u8,
    // TODO remove `allow`
    #[allow(dead_code)] ppre2: u8,
    sysclk: Hertz,
}

impl Clocks {
    /// Returns the frequency of the AHB
    pub fn hclk(&self) -> Hertz {
        self.hclk
    }

    /// Returns the frequency of the APB1
    pub fn pclk1(&self) -> Hertz {
        self.pclk1
    }

    /// Returns the frequency of the APB2
    pub fn pclk2(&self) -> Hertz {
        self.pclk2
    }

    pub(crate) fn ppre1(&self) -> u8 {
        self.ppre1
    }

    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn ppre2(&self) -> u8 {
        self.ppre2
    }

    /// Returns the system (core) frequency
    pub fn sysclk(&self) -> Hertz {
        self.sysclk
    }
}
