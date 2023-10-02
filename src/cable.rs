//! Implementations for different JTAG hardware adapters will here.  Hardware adapters should
//! implement the `Cable` trait.
pub mod jtagkey;

pub trait Cable {
    /// Clock out a series of TMS values to change the state of the JTAG chain.  Each element of
    /// `tms` determines the value of the TMS line, zero for low and any other value for high.
    /// `tdo` controls the state of the TDI line during mode changes.
    fn change_mode(&mut self, tms: &[usize], tdo: bool);
    /// Shift in bits from the TDO line.  `bits` is the total number of bits to read.  Should be
    /// called with state = ShiftIR or ShiftDR, and will remain in that state.  Should clock out
    /// all ones.
    fn read_data(&mut self, bits: usize) -> Vec<u8>;
    /// Shift out bits on the TDI line.  `bits` is the number of bits to send from the last byte.
    /// Should be called with state = ShiftIR or ShiftDR.  State won't change unless `pause_after`
    /// is true, in which case it will be PauseIR or PauseDR on exit.
    fn write_data(&mut self, data: &[u8], bits: u8, pause_after: bool);

    fn read_write_data(&mut self, data: &[u8], bits: u8, pause_after: bool) -> Vec<u8>;
}
