//! Watchdog peripherals

use stm32f429::IWDG;

/// Watchdogs will reset your device if it fails reloading them.
pub trait Watchdog {
    /// Write reload value to watchdog
    fn reload(&mut self);
}

/// Wraps the Independent Watchdog (IWDG) peripheral
pub struct IndependentWatchdog {
    iwdg: IWDG,
}

const MAX_PR: u8 = 6;
const MAX_RL: u16 = 0xFFF;
const KR_ACCESS: u16 = 0x5555;
const KR_RELOAD: u16 = 0xAAAA;
const KR_START: u16 = 0xCCCC;


impl IndependentWatchdog {
    /// Wrap and start the watchdog
    pub fn new(iwdg: IWDG, timeout_ms: u32) -> Self {
        IndependentWatchdog { iwdg }
            .setup(timeout_ms)
    }

    fn setup(self, timeout_ms: u32) -> Self {
        let mut pr = 0;
        while pr < MAX_PR && Self::timeout_period(pr, MAX_RL) < timeout_ms {
            pr += 1;
        }

        let max_period = Self::timeout_period(pr, MAX_RL);
        let max_rl = u32::from(MAX_RL);
        let rl = (timeout_ms * max_rl / max_period).min(max_rl) as u16;

        self.access_registers(|iwdg| {
            iwdg.pr.modify(|_,w| unsafe { w.pr().bits(pr) });
            iwdg.rlr.modify(|_, w| unsafe { w.rl().bits(rl) });
        });
        self.iwdg.kr.write(|w| unsafe { w.key().bits(KR_START) });

        self
    }

    fn is_pr_updating(&self) -> bool {
        self.iwdg.sr.read().pvu().bit()
    }

    /// Returns the interval in ms
    pub fn interval(&self) -> u32 {
        while self.is_pr_updating() {}

        let pr = self.iwdg.pr.read().pr().bits();
        let rl = self.iwdg.rlr.read().rl().bits();
        Self::timeout_period(pr, rl)
    }

    /// pr: Prescaler divider bits, rl: reload value
    ///
    /// Returns ms
    fn timeout_period(pr: u8, rl: u16) -> u32 {
        let divider: u32 = [4, 8, 16, 32, 64, 128, 256][usize::from(pr)];
        (u32::from(rl) + 1) * divider / 32
    }

    fn access_registers<A, F: FnMut(&IWDG) -> A>(&self, mut f: F) -> A {
        // Unprotect write access to registers
        self.iwdg.kr.write(|w| unsafe { w.key().bits(KR_ACCESS) });
        let a = f(&self.iwdg);

        // Protect again
        // self.iwdg.kr.write(|w| unsafe { w.key().bits(KR_RELOAD) });
        a
    }
}

impl Watchdog for IndependentWatchdog {
    fn reload(&mut self) {
        self.iwdg.kr.write(|w| unsafe { w.key().bits(KR_RELOAD) });
    }
}
