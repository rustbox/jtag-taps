//! This crate allows for interacting with a JTAG test chain at a variety of levels
//! of abstraction.  At the lowest level, you can directly interact with a JTAG
//! cable, such as those based on the "jtagkey" design.  Other FT2232-based cables
//! could easily be added, as well as direct GPIO control.  The Cable trait allows
//! for changing modes and shifting bits in and out of the JTAG chain.
//! 
//! The next higher level of abstraction is the JtagSM, which keeps track of the
//! mode of the TAPs.  You tell it which mode you want (e.g., Reset or Idle) and it
//! gets there with the fewest number of mode changes.  You can also read and write
//! the instruction and data registers.  read_write and write_reg take care of
//! getting to ShiftDR or ShiftIR mode as appropriate.
//! 
//! If there are multiple TAPs in the JTAG chain, you probably want to use the
//! methods associated with the Taps struct.  You tell Taps how much TAPs exist and
//! what the IR length is for each one.  You can then read and write the
//! instruction and data registers for that one TAP, and it will take care of
//! putting other TAPs in BYPASS and shifting data in and out as appropriate.  Taps
//! also has some support for automatically detecting the IR lengths and ID codes
//! of the TAPs.
//! 
//! # Example
//! ```
//! use jtag_taps::cable::mpsse::JtagKey;
//! use jtag_taps::statemachine::JtagSM;
//! use jtag_taps::taps::Taps;
//! let cable = JtagKey::new("Dual RS232-HS A", 1 << 20);
//! let jtag = JtagSM::new(Box::new(cable));
//! let mut taps = Taps::new(jtag);
//! taps.detect();
//! 
//! let ir = vec![235, 0];
//! taps.select_tap(0, &ir);
//! let buf = vec![
//!     0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
//! ];
//! taps.write_dr(&buf, 8);
//! ```


#![no_std]

#[cfg(feature = "std")]
extern crate std;

extern crate alloc;

pub mod cable;
pub mod statemachine;
pub mod taps;
