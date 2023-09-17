//! A convenience wrapper for JTAG scan chains with multiple TAPs present.  `Taps` allows the
//! client to interact with one selected TAP as if it were the only TAP in the chain, so that the
//! client doesn't have to deal with putting the other TAPs into bypass and shifting data through
//! the bypass registers.
use crate::statemachine::{JtagSM, Register};

fn shift_left(input: &[u8], mut shift: usize) -> Vec<u8> {
    let bytes = shift / 8;
    shift %= 8;
    let mut output = vec![0xff; bytes];

    let mut remainder = (1 << shift) - 1;
    for x in input {
        if shift > 0 {
            let top = *x >> (8 - shift);
            let mut new = *x << shift;
            new |= remainder;
            remainder = top;
            output.push(new);
        } else {
            output.push(*x);
        }
    }
    output
}

struct Tap {
    irlen: usize,
}

pub struct Taps {
    pub sm: JtagSM,
    taps: Vec<Tap>,
    active: usize,
}

impl Taps {
    /// Create an object using an existing `JtagSM` object
    pub fn new(sm: JtagSM) -> Self {
        Self {
            sm,
            taps: Vec::new(),
            active: 0,
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
        let ir = shift_left(ir, pad_bits);
        self.sm.write_reg(Register::Instruction, &ir, total_bits as u8, true);
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
        self.sm.read_reg(Register::Instruction, pad_bits);
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
        let dr = shift_left(dr, pad_bits);
        self.sm.write_reg(Register::Data, &dr, total_bits as u8, true);
    }

    /// Read the data register of the TAP selected by `select_tap`.  `bits` indicates the length of
    /// the data register for the current instruction.
    pub fn read_dr(&mut self, bits: usize) -> Vec<u8> {
        assert!(self.active < self.taps.len());
        let pad_bits = self.taps.len() - self.active - 1;

        // Discard the bypass bits
        self.sm.read_reg(Register::Data, pad_bits);
        self.sm.read_reg(Register::Data, bits)
    }
}

