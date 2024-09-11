use alloc::vec::Vec;
use alloc::vec;
use embedded_hal::{delay::DelayNs, digital::{InputPin, OutputPin, PinState}};

use crate::cable::Cable;

pub struct Gpio<Clk, Tdi, Tdo, Tms, Delay> where Clk: OutputPin, Tdi: OutputPin, Tdo: InputPin, Tms: OutputPin, Delay: DelayNs {
    half_period: u32,
    delay: Delay,
    clock: Clk,
    tdi: Tdi,
    tdo: Tdo,
    tms: Tms
}

impl<Clk, Tdi, Tdo, Tms, Delay> Gpio<Clk, Tdi, Tdo, Tms, Delay> where Clk: OutputPin, Tdi: OutputPin, Tdo: InputPin, Tms: OutputPin, Delay: DelayNs {
    pub fn new(freq_khz: u32, clock: Clk, tdi: Tdi, tdo: Tdo, tms: Tms, delay: Delay) -> Gpio<Clk, Tdi, Tdo, Tms, Delay> {
        let period_ns = 1_000_000 / freq_khz;
        let half_period = period_ns / 2;
        Gpio { half_period, clock, tdi, tdo, tms, delay }
    }
}

impl<Clk, Tdi, Tdo, Tms, Delay> Cable for Gpio<Clk, Tdi, Tdo, Tms, Delay> where Clk: OutputPin, Tdi: OutputPin, Tdo: InputPin, Tms: OutputPin, Delay: DelayNs {
    fn change_mode(&mut self, tms: &[usize], tdo: bool) {
        // clock starts low
        self.tdi.set_state(PinState::from(tdo)).unwrap();

        for d in tms {
            let state = match d {
                0 => PinState::Low,
                _ => PinState::High,
            };
            self.tms.set_state(state).unwrap();
            self.clock.set_high().unwrap();

            self.delay.delay_ns(self.half_period);
            self.clock.set_low().unwrap();
            self.delay.delay_ns(self.half_period);
        }
    }

    fn read_data(&mut self, bits: usize) -> Vec<u8> {
        let mut buf = vec![];
        let mut value: u8 = 0;
        let mut b = 0;
        for _ in 0..bits {
            self.clock.set_high().unwrap();
            // Sample the tdo line
            let bit = self.tdo.is_high().unwrap() as u8;

            // Shift in the bit into the next byte
            value = value | (bit << b);
            b = (b + 1) % 8;

            // When we get back to 0, we've finished a byte
            if b == 0 {
                buf.push(value);
            }

            // Finish the clock period
            self.delay.delay_ns(self.half_period);
            self.clock.set_low().unwrap();
            self.delay.delay_ns(self.half_period);
        }
        // If we have anything left over, push it onto buf incomplete
        buf.push(value);
        buf
    }

    fn write_data(&mut self, data: &[u8], bits: u8, pause_after: bool) {
        self.read_write_data(data, bits, pause_after);
    }

    fn read_write_data(&mut self, data: &[u8], bits: u8, pause_after: bool) -> Vec<u8> {
        // data: [d0, d1, ..., d_n-1]
        // we go up to d_n-1, and send `bits` number of bits from d_n-1
        
        // Constrain `bits` to be between 1 and 8
        let bits = bits.max(1).min(8);

        let mut out_buffer = vec![];

        for d in &data[0..data.len()-1] {
            let mut byte = 0;
            for b in 0..8 {
                
                // Write a bit from `data` onto TDI, high to low
                let tdi = (d >> (7 - b)) & 1 == 1;
                self.tdi.set_state(PinState::from(tdi)).unwrap();

                // Clock high
                self.clock.set_high().unwrap();

                // Sample a bit from TDO, low to high
                let tdo = self.tdo.is_high().unwrap() as u8;
                byte = byte | (tdo << b);

                // Wait and clock low, finishing the clock cycle
                self.delay.delay_ns(self.half_period);
                self.clock.set_low().unwrap();
                self.delay.delay_ns(self.half_period);
            }
            // Once we do 8 bits, push the read byte into the buffer
            out_buffer.push(byte);
        }

        // Handle the last partial byte
        let d = &data[data.len() - 1];
        let mut byte = 0;
        for b in 0..bits {
            // Write a bit from `data` onto TDI, high to low
            let tdi = (d >> (7 - b)) & 1 == 1;
            self.tdi.set_state(PinState::from(tdi)).unwrap();

            if b == bits - 1 && pause_after {
                // If we're on the last bit of the read/write 
                // and we're supposed to pause after, then activate TMS
                self.tms.set_high().unwrap();
            }

            // Clock high
            self.clock.set_high().unwrap();

            // Sample a bit from TDO, low to high
            let tdo = self.tdo.is_high().unwrap() as u8;
            byte = byte | (tdo << b);

            // Wait and clock low, finishing the clock cycle
            self.delay.delay_ns(self.half_period);
            self.clock.set_low().unwrap();
            self.delay.delay_ns(self.half_period);
        }
        out_buffer.push(byte);
        out_buffer

    }

    fn queue_read(&mut self, _bits: usize) -> bool {
        // We won't support this for the moment
        false
    }

    fn queue_read_write(&mut self, _data: &[u8], _bits: u8, _pause_after: bool) -> bool {
        // We won't support this for the moment
        false
    }

    fn finish_read(&mut self, _bits: usize) -> Vec<u8> {
        // not supported
        vec![]
    }
}
