//! A convenience wrapper for JTAG scan chains with multiple TAPs present.  `Taps` allows the
//! client to interact with one selected TAP as if it were the only TAP in the chain, so that the
//! client doesn't have to deal with putting the other TAPs into bypass and shifting data through
//! the bypass registers.
use crate::statemachine::{JtagSM, JtagState, Register};
use crate::cable::Cable;

fn add_ones_to_end(input: &[u8], this_len: usize, shift: usize) -> Vec<u8> {
    let bytes = shift / 8;
    let mut output = input.to_vec();

    let top_bits = (1 << (this_len % 8)) - 1;
    let end = output.len()-1;
    output[end] |= !top_bits;

    let mut pad = vec![0xff; bytes];
    output.append(&mut pad);
    output
}

struct Tap {
    irlen: usize,
}

pub struct Taps<T> {
    pub sm: JtagSM<T>,
    taps: Vec<Tap>,
    active: usize,
    dangling_read: bool,
    queued_reads: usize
}

impl<T, U> Taps<T>
    where T: std::ops::DerefMut<Target=U>,
          U: Cable + ?Sized
{
    /// Create an object using an existing `JtagSM` object
    pub fn new(sm: JtagSM<T>) -> Self {
        Self {
            sm,
            taps: Vec::new(),
            active: 0,
            dangling_read: false,
            queued_reads: 0,
        }
    }

    /// Add a TAP to the scan chain with the given instruction register length
    pub fn add_tap(&mut self, irlen: usize) {
        let tap = Tap {
            irlen
        };
        self.taps.push(tap);
    }

    /// Attempt to autodetect the number of TAPs on the scan chain and the instruction register
    /// length for each.
    pub fn detect(&mut self) {
        self.taps = Vec::new();
        self.sm.mode_reset();

        let mut count: i32 = -1;
        let mut irlen = vec![];
        loop {
            let bit = self.sm.read_reg(Register::Instruction, 1);
            if bit[0] != 0 {
                if count > 0 {
                    println!("found IR len {}", count+1);
                    irlen.push(count+1)
                }
                if count == 0 {
                    break;
                }
                count = 0;
            } else {
                count += 1;
            }
        }

        self.sm.mode_reset();
        let mut ids = vec![];
        for _ in 0..irlen.len() {
            let bit = self.sm.read_reg(Register::Data, 1);
            if bit[0] == 0 {
                println!("invalid IDCODE of 0");
                ids.push(0);
            } else {
                let bits = self.sm.read_reg(Register::Data, 31);
                let idcode = u32::from_le_bytes(bits.try_into().unwrap());
                // Add back the one we read
                ids.push((idcode << 1) | 1);
            }
        }

        irlen.reverse();
        ids.reverse();

        for i in 0..irlen.len() {
            println!("Adding tap {} idcode {:x}", i, ids[i]);
            self.add_tap(irlen[i] as usize);
        }
    }

    /// Select which TAP in the scan chain to operate upon.  `ir` will be shifted into its
    /// instruction register, and the other TAPs put into bypass.
    pub fn select_tap(&mut self, tap: usize, ir: &[u8]) {
        assert!(tap < self.taps.len());
        self.sm.mode_reset();
        self.active = tap;
        self.write_ir(ir);
    }

    fn write_ones(&mut self, mut bits: usize) {
        let bytes = bits / 8;
        bits %= 8;

        if bytes > 0 {
            let buf = vec![0xff; bytes];
            self.sm.write_reg(Register::Instruction, &buf, 8, false);
        }
        if bits > 0 {
            let buf = vec![(1 << bits) - 1];
            self.sm.write_reg(Register::Instruction, &buf, bits as u8, false);
        }
    }

    /// Shift `ir` into the instruction register of the TAP selected by `select_tap`
    pub fn write_ir(&mut self, ir: &[u8]) {
        assert!(self.active < self.taps.len());
        let this_irlen = self.taps[self.active].irlen;
        assert_eq!(ir.len(), (this_irlen + 7) / 8);

        // Put downstream taps into BYPASS
        let mut after_pad = 0;
        for t in &self.taps[self.active+1..] {
            after_pad += t.irlen;
        }
        self.write_ones(after_pad);

        let mut pad_bits = 0;
        for t in &self.taps[0..self.active] {
            pad_bits += t.irlen;
        }
        let mut total_bits = (pad_bits + this_irlen) % 8;
        if total_bits == 0 {
            total_bits = 8;
        }
        let ir = add_ones_to_end(ir, this_irlen, pad_bits);
        self.sm.write_reg(Register::Instruction, &ir, total_bits as u8, true);
        self.sm.change_mode(JtagState::Idle);
    }

    /// Read the instruction register of the TAP selected by `select_tap`
    pub fn read_ir(&mut self) -> Vec<u8> {
        assert!(self.active < self.taps.len());
        let this_irlen = self.taps[self.active].irlen;
        let mut pad_bits = 0;
        for t in &self.taps[self.active+1..] {
            pad_bits += t.irlen;
        }

        // Discard the unwanted bits
        self.sm.change_mode(JtagState::Idle);
        if pad_bits > 0 {
            self.sm.read_reg(Register::Instruction, pad_bits);
        }
        self.sm.read_reg(Register::Instruction, this_irlen)
    }

    /// Shift `dr` into the data register of the TAP selected by `select_tap`.  `bits` indicates
    /// how many bits of the final byte should be written (a value of 8 will write the entire byte)
    pub fn write_dr(&mut self, dr: &[u8], bits: usize) {
        assert!(self.active < self.taps.len());
        let this_len = (dr.len() - 1) * 8 + bits;
        let pad_bits = self.active;

        let mut total_bits = (pad_bits + this_len) % 8;
        if total_bits == 0 {
            total_bits = 8;
        }
        let dr = add_ones_to_end(dr, this_len, pad_bits);
        self.sm.write_reg(Register::Data, &dr, total_bits as u8, true);
        self.sm.change_mode(JtagState::Idle);
    }

    /// Shift `dr` into the data register of the TAP selected by `select_tap`.  `bits` indicates
    /// how many bits of the final byte should be written (a value of 8 will
    /// write the entire byte).  Returns the bits that were shifted out while `dr` was
    /// shifted in.
    pub fn read_write_dr(&mut self, dr: &[u8], bits: usize) -> Vec<u8> {
        assert!(self.active < self.taps.len());
        let this_len = (dr.len() - 1) * 8 + bits;
        let pad_bits = self.active;
        let discard_bits = self.taps.len() - self.active - 1;

        let mut total_bits = (pad_bits + this_len) % 8;
        if total_bits == 0 {
            total_bits = 8;
        }
        let dr = add_ones_to_end(dr, this_len, pad_bits);
        if discard_bits > 0 {
            self.sm.read_reg(Register::Data, discard_bits);
        }
        let data = self.sm.read_write_reg(Register::Data, &dr, total_bits as u8, true);
        self.sm.change_mode(JtagState::Idle);
        data
    }

    /// Read the data register of the TAP selected by `select_tap`.  `bits` indicates the length of
    /// the data register for the current instruction.
    pub fn read_dr(&mut self, bits: usize) -> Vec<u8> {
        assert!(self.active < self.taps.len());
        let pad_bits = self.taps.len() - self.active - 1;

        // Discard the bypass bits
        self.sm.change_mode(JtagState::Idle);
        if pad_bits > 0 {
            self.sm.read_reg(Register::Data, pad_bits);
        }
        self.sm.read_reg(Register::Data, bits)
    }

    pub fn queue_dr_read(&mut self, bits: usize) -> bool {
        assert!(self.active < self.taps.len());
        let pad_bits = self.active;
        let discard_bits = self.taps.len() - self.active - 1;
        let total_bits = pad_bits + bits;

        // Discard the bypass bits
        self.sm.change_mode(JtagState::Idle);
        if discard_bits > 0 {
            if !self.sm.queue_read(Register::Data, discard_bits) {
                return false;
            }
        }
        if !self.sm.queue_read(Register::Data, total_bits) {
            self.dangling_read = true;
            false
        } else {
            self.queued_reads += 1;
            true
        }
    }

    pub fn finish_dr_read(&mut self, bits: usize) -> Vec<u8> {
        assert!(self.active < self.taps.len());
        let pad_bits = self.active;
        let discard_bits = self.taps.len() - self.active - 1;
        let total_bits = pad_bits + bits;

        // Discard the bypass bits
        if discard_bits > 0 {
            self.sm.cable.finish_read(discard_bits);
        }
        let ret = self.sm.cable.finish_read(total_bits);

        // Handle the case where we were able to queue the read of the discard bits, but not of the
        // interesting data.
        self.queued_reads -= 1;
        if self.queued_reads == 0 && self.dangling_read {
            self.sm.cable.finish_read(discard_bits);
            self.dangling_read = false;
        }
        ret
    }
}

