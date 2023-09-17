jtag-taps
=========
This crate allows for interacting with a JTAG test chain at a variety of levels
of abstraction.  At the lowest level, you can directly interact with a JTAG
cable, such as those based on the "jtagkey" design.  Other FT2232-based cables
could easily be added, as well as direct GPIO control.  The Cable trait allows
for changing modes and shifting bits in and out of the JTAG chain.

The next higher level of abstraction is the JtagSM, which keeps track of the
mode of the TAPs.  You tell it which mode you want (e.g., Reset or Idle) and it
gets there with the fewest number of mode changes.  You can also read and write
the instruction and data registers.  read_write and write_reg take care of
getting to ShiftDR or ShiftIR mode as appropriate.

If there are multiple TAPs in the JTAG chain, you probably want to use the
methods associated with the Taps struct.  You tell Taps how much TAPs exist and
what the IR length is for each one.  You can then read and write the
instruction and data registers for that one TAP, and it will take care of
putting other TAPs in BYPASS and shifting data in and out as appropriate.  Taps
also has some support for automatically detecting the IR lengths and ID codes
of the TAPs.
