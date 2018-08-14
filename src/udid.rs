//! Reading the Unique Device ID register

use stm32f429::DEVICE_ID;

/// Implemented for the `DEVICE_ID` peripheral.
pub trait DeviceIdExt {
    /// Read X and Y coordinates on the wafer.
    ///
    /// Decodes the ASCII encoded digits
    fn read_xy(&self) -> Option<(u32, u32)>;
    /// Read Wafer number
    ///
    /// Supposedly ASCII encoded digits, but mine yields ASCII code
    /// 22. Therefore this is the raw byte for now.
    fn read_waf_num(&self) -> u8;
    /// Read Lot number
    ///
    /// ASCII encoded digits. Mine has a trailing 'Q' which gets
    /// ignored by `ReadAscii`.
    fn read_lot_num(&self) -> Option<u32>;
}

impl DeviceIdExt for DEVICE_ID {
    fn read_xy(&self) -> Option<(u32, u32)> {
        let uid1 = self.uid1.read().uid().bits();
        let x = ((uid1 >> 16) as u16).read_ascii()?;
        let y = (uid1 as u16).read_ascii()?;
        Some((x, y))
    }

    fn read_waf_num(&self) -> u8 {
        self.uid2.read().waf_num().bits()
    }

    fn read_lot_num(&self) -> Option<u32> {
        let lot_num =
            (self.uid2.read().lot_num().bits() as u64) |
            ((self.uid3.read().lot_num().bits() as u64) << 24);
        lot_num.read_ascii()
    }
}

trait ReadAscii {
    type B: AsRef<[u8]>;

    fn as_bytes(&self) -> Self::B;
    fn read_ascii(&self) -> Option<u32> {
        let mut r = 0;
        for (i, c) in self.as_bytes().as_ref().iter().cloned().enumerate() {
            if i == 0 && c == 0 {
                // skip \0
            } else if c >= '0' as u8 && c <= '9' as u8 {
                let digit: u8 = (c as u8) - ('0' as u8);
                r = (r * 10) + digit as u32;
            } else if i == 0 {
                return None;
            }
        }
        Some(r)
    }
}

impl ReadAscii for u16 {
    type B = [u8; 2];

    fn as_bytes(&self) -> Self::B {
        [(*self >> 8) as u8, *self as u8]
    }
}

impl ReadAscii for u64 {
    type B = [u8; 8];

    fn as_bytes(&self) -> Self::B {
        [(*self >> 56) as u8, (*self >> 48) as u8,
         (*self >> 40) as u8, (*self >> 32) as u8,
         (*self >> 24) as u8, (*self >> 16) as u8,
         (*self >> 8) as u8, *self as u8]
    }
}
