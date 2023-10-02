//! Implement the `Cable` trait for "jtagkey" compatible hardware adapters like the Bus Blaster
use crate::cable::Cable;

use libftd2xx::{Ft2232h, Ftdi, FtdiMpsse, MpsseCmdBuilder, MpsseCmdExecutor};
use ftdi_mpsse::ClockTMSOut;
use libftd2xx::{ClockData, ClockDataOut, ClockBits, ClockBitsOut};

pub struct Mpsse<T> {
    ft: T,
}

impl<T: FtdiMpsse + MpsseCmdExecutor> Mpsse<T>
    where <T as MpsseCmdExecutor>::Error: std::fmt::Debug
{
    pub fn new(mut ft: T, clock: u32) -> Self
    {
        ft.initialize_mpsse_default().expect("init");
        ft.set_clock(clock).expect("set clock");

        let builder = MpsseCmdBuilder::new()
            .disable_3phase_data_clocking()
            .disable_adaptive_data_clocking();
        ft.send(builder.as_slice()).expect("send");

        Self {
            ft,
        }
    }
}

impl<T: FtdiMpsse + MpsseCmdExecutor> Cable for Mpsse<T>
    where <T as MpsseCmdExecutor>::Error: std::fmt::Debug
{
    fn change_mode(&mut self, tms: &[usize], tdo: bool) {
        let mut count = 0;
        let mut buf = 0;
        let mut builder = MpsseCmdBuilder::new();

        for x in tms {
            if *x != 0 {
                buf |= 1 << count;
            }
            count += 1;

            if count == 7 {
                builder = builder.clock_tms_out(ClockTMSOut::NegEdge, buf, tdo, count);
                count = 0;
                buf = 0;
            }
        }
        builder = builder.clock_tms_out(ClockTMSOut::NegEdge, buf, tdo, count);
        self.ft.send(builder.as_slice()).expect("send");
    }

    fn read_data(&mut self, mut bits: usize) -> Vec<u8>
    {
        let mut bytes = bits / 8;
        let mut builder = MpsseCmdBuilder::new();
        if bytes > 0 {
            bits -= bytes * 8;
            builder = builder.clock_data(ClockData::LsbPosIn, &vec![0xff; bytes]);
        }

        if bits > 0 {
            builder = builder.clock_bits(ClockBits::LsbPosIn, 0xff, bits as u8);
            bytes += 1;
        }
        let mut buf = vec![0; bytes];
        self.ft.xfer(builder.as_slice(), &mut buf).expect("send");
        if bits > 0 {
            let last_idx = buf.len()-1;
            buf[last_idx] >>= 8 - bits;
        }
        buf
    }

    fn write_data(&mut self, data: &[u8], mut bits: u8, pause_after: bool)
    {
        let mut builder = MpsseCmdBuilder::new();
        //
        // We will send the last bit using clock_tms
        assert!(bits <= 8);
        assert!(bits != 0);

        bits -= 1;

        if data.len() > 1 {
            builder = builder.clock_data_out(ClockDataOut::LsbNeg, &data[..data.len()-1]);
        }
        let last_byte = data[data.len()-1];
        if bits > 1 {
            builder = builder.clock_bits_out(ClockBitsOut::LsbNeg, last_byte, bits);
        }
        let last_bit = last_byte & (1 << bits) != 0;
        // Change to pause state
        if pause_after {
            builder = builder.clock_tms_out(ClockTMSOut::NegEdge, 1, last_bit, 2);
        } else {
            builder = builder.clock_tms_out(ClockTMSOut::NegEdge, 0, last_bit, 1);
        }

        self.ft.send(builder.as_slice()).expect("send");
    }

    fn read_write_data(&mut self, _data: &[u8], _bits: u8, _pause_after: bool) -> Vec<u8> {
        unimplemented!();
    }
}

// Lower pins
const PIN_TCK: u8 = 1;
const PIN_TDI: u8 = 1 << 1;
//const PIN_TDO: u8 = 1 << 2;
const PIN_TMS: u8 = 1 << 3;
const PIN_N_OE: u8 = 1 << 4;
const LOWER_OUTPUT_PINS: u8 = PIN_TCK | PIN_TDI | PIN_TMS | PIN_N_OE;

// Upper pins
const PIN_N_TRST: u8 = 1;
const PIN_N_SRST: u8 = 1 << 1;
const PIN_N_TRST_OE: u8 = 1 << 2;
const PIN_N_SRST_OE: u8 = 1 << 3;
const UPPER_OUTPUT_PINS: u8 = PIN_N_TRST | PIN_N_SRST | PIN_N_TRST_OE | PIN_N_SRST_OE;

pub struct JtagKey {
    ft: Mpsse<Ft2232h>,
}

impl JtagKey {
    /// Create a new JtagKey.  FT2232-based adapters like JtagKey have both an "A" interface and a
    /// "B" interface.  `primary` controls which to use. `clock` controls the speed of TCLK in hertz.
    pub fn new(clock: u32, primary: bool) -> Self {
        let description = if primary {
            "Dual RS232-HS A"
        } else {
            "Dual RS232-HS B"
        };
        let ft = Ftdi::with_description(description).expect("new");
        let mut ft = Ft2232h::try_from(ft).expect("try");
        ft.set_gpio_upper(PIN_N_TRST | PIN_N_SRST, UPPER_OUTPUT_PINS).expect("pins");
        let mut ft = Mpsse::new(ft, clock);

        let builder = MpsseCmdBuilder::new()
            .set_gpio_lower(PIN_TMS, LOWER_OUTPUT_PINS);
        ft.ft.send(builder.as_slice()).expect("send");

        JtagKey {
            ft
        }
    }

    /// JtagKey adapters implement the option SRST signal.  This function puts the system in reset.
    pub fn assert_srst(&mut self) {
        self.ft.ft.set_gpio_upper(PIN_N_TRST, UPPER_OUTPUT_PINS).expect("pins");
    }

    /// JtagKey adapters implement the option SRST signal.  This function takes the system out of
    /// reset.
    pub fn dessert_srst(&mut self) {
        self.ft.ft.set_gpio_upper(PIN_N_TRST | PIN_N_SRST, UPPER_OUTPUT_PINS).expect("pins");
    }
}

impl Cable for JtagKey {
    fn change_mode(&mut self, tms: &[usize], tdo: bool) {
        self.ft.change_mode(tms, tdo)
    }

    fn read_data(&mut self, bits: usize) -> Vec<u8> {
        self.ft.read_data(bits)
    }

    fn write_data(&mut self, data: &[u8], bits: u8, pause_after: bool) {
        self.ft.write_data(data, bits, pause_after)
    }

    fn read_write_data(&mut self, data: &[u8], bits: u8, pause_after: bool) -> Vec<u8> {
        self.ft.read_write_data(data, bits, pause_after)
    }
}
