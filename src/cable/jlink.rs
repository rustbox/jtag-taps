//! Implement the `Cable` trait for "jlink" compatible hardware adapters
use crate::cable::Cable;

use std::time::Duration;

use rusb::{DeviceHandle, Direction, GlobalContext};
use rusb::constants::*;

pub struct JLink {
    device: DeviceHandle<GlobalContext>,
    // queued bytes to send
    buffer: Vec<u8>,
    // number of bytes we'll receive after sending the above
    recv_bytes: usize,
    // queued tms changes
    tms_buf: Vec<u8>,
    // queued tdo changes
    tdo_buf: Vec<u8>,
    // number of bits in the above
    send_bits: usize,
    read_endpoint: u8,
    write_endpoint: u8,
    read_queue: Vec<Vec<u8>>,
}

fn bit_append (dst: &mut Vec<u8>, mut dst_bits: usize, src: &mut [u8], src_bits: usize, src_skip: usize) {
    let mut byte = if !dst.is_empty() && dst_bits % 8 != 0 {
        dst.pop().unwrap()
    } else {
        0
    };

    for i in src_skip..src_bits {
        if src[i / 8] & (1 << (i % 8)) != 0{
            byte |= 1 << (dst_bits % 8);
        }

        dst_bits += 1;
        if dst_bits % 8 == 0 {
            dst.push(byte);
            byte = 0;
        }
    }
    if dst_bits % 8 != 0 {
        dst.push(byte);
    }
}

impl JLink {
    pub fn new(clock: u32) -> Self {
        let device = rusb::open_device_with_vid_pid(0x1366, 0x0105).expect("no jlink attached");
        let descriptor = device.device().active_config_descriptor().expect("active config");
        for i in descriptor.interfaces() {
            for d in i.descriptors() {
                if d.class_code() != LIBUSB_CLASS_VENDOR_SPEC ||
                    d.sub_class_code() != LIBUSB_CLASS_VENDOR_SPEC ||
                        d.num_endpoints() < 2 {
                            continue;
                }

                let mut read_endpoint = None;
                let mut write_endpoint = None;
                for e in d.endpoint_descriptors() {
                    match e.direction() {
                        Direction::In => read_endpoint = Some(e.address()),
                        Direction::Out => write_endpoint = Some(e.address()),
                    }
                }

                if read_endpoint.is_none() || write_endpoint.is_none() {
                    continue;
                }

                let read_endpoint = read_endpoint.unwrap();
                let write_endpoint = write_endpoint.unwrap();

                let mut buf = [0; 2];
                let _ = device.read_bulk(read_endpoint, &mut buf, Duration::from_millis(10));

                let mut jlink = Self {
                    device,
                    buffer: vec![],
                    tms_buf: vec![],
                    tdo_buf: vec![],
                    read_queue: vec![],
                    send_bits: 0,
                    recv_bytes: 0,
                    read_endpoint,
                    write_endpoint,
                };

                jlink.get_status();
                jlink.set_clock(clock);
                jlink.set_interface(0);
                jlink.deassert_trst();
                jlink.deassert_srst();

                return jlink;
            }
        }
        panic!("no jlink attached");
    }

    fn send_command(&mut self, cmd: u8, mut data: Vec<u8>) {
        self.flush_tap_sequence();
        data.insert(0, cmd);
        self.buffer.append(&mut data);
    }

    fn read_data(&mut self, len: usize) -> Result<Vec<u8>, rusb::Error> {
        // Submit any pending writes
        self.flush_tap_sequence();
        let wr = self.device.write_bulk(self.write_endpoint, &self.buffer, Duration::from_millis(100))?;
        assert_eq!(wr, self.buffer.len());
        self.buffer.clear();

        let mut recv_bytes = len + self.recv_bytes;
        let mut data = vec![];

        while recv_bytes > 0 {
            let mut buffer = vec![0; recv_bytes];
            let len = self.device.read_bulk(self.read_endpoint, &mut buffer, Duration::from_millis(100))?;
            buffer.resize(len, 0);
            data.append(&mut buffer);
            recv_bytes -= len;
        }

        // Don't return any of the data from the pending write that we didn't care about
        let data = data.split_off(self.recv_bytes);
        self.recv_bytes = 0;
        Ok(data)
    }

    pub fn get_status(&mut self) -> Vec<u8> {
        self.send_command(0x7, vec![]);
        let data = self.read_data(8).expect("read status");

        let vref = ((data[0] as u16) + (data[1] as u16)) << 8;
        if vref < 15000 {
            panic!("vref too low, possibly unpowered or disconnected");
        }
        data
    }

    pub fn set_clock(&mut self, mut clock: u32) {
        clock /= 1000;
        let buf = vec![(clock & 0xff) as u8, ((clock >> 8) & 0xff) as u8];
        self.send_command(0x5, buf);
    }

    pub fn set_interface(&mut self, intf: u8) {
        let buf = vec![intf];
        self.send_command(0xc7, buf);
        let _ = self.read_data(4).expect("set interface response");
    }

    pub fn assert_srst(&mut self) {
        self.send_command(0xdc, vec![]);
    }

    pub fn deassert_srst(&mut self) {
        self.send_command(0xdd, vec![]);
    }

    pub fn assert_trst(&mut self) {
        self.send_command(0xde, vec![]);
    }

    pub fn deassert_trst(&mut self) {
        self.send_command(0xdf, vec![]);
    }

    fn tap_sequence(&mut self, mut tms: Vec<u8>, mut tdo: Vec<u8>, bits: usize) {
        assert_eq!(tms.len(), tdo.len());
        assert!(tms.len() < 390);

        bit_append(&mut self.tms_buf, self.send_bits, &mut tms, bits, 0);
        bit_append(&mut self.tdo_buf, self.send_bits, &mut tdo, bits, 0);
        self.send_bits += bits;
    }

    fn flush_tap_sequence(&mut self) {
        assert_eq!(self.tms_buf.len(), self.tdo_buf.len());
        assert!(self.tms_buf.len() < 390);
        if !self.tms_buf.is_empty() {
            let bytes = self.tms_buf.len();
            let mut cmdbuf = vec![0xcd, (self.send_bits & 0xff) as u8, ((self.send_bits >> 8) & 0xff) as u8];
            cmdbuf.append(&mut self.tms_buf);
            cmdbuf.append(&mut self.tdo_buf);
            self.send_bits = 0;

            self.buffer.append(&mut cmdbuf);
            // queue bytes for later read, rather than reading them now
            self.recv_bytes += bytes;
        }
    }

    fn send_tdo(&mut self, data: &[u8], bits: u8, pause_after: bool) {
        let mut total_bits = (data.len()-1) * 8 + (bits as usize);

        let mut tms = vec![0; data.len()];
        let mut data = data.to_vec();

        if pause_after {
            let len = tms.len();
            tms[len-1] |= 1 << (bits-1);

            // Add an extra clock for the transition to pause state
            if total_bits % 8 == 0 {
                data.push(0xff);
                tms.push(0);
            }
            total_bits += 1;
        }

        self.tap_sequence(tms, data, total_bits);
    }
}

impl Cable for JLink {
    fn change_mode(&mut self, tms: &[usize], tdo: bool) {
        let mut buf = vec![];
        let mut byte = 0u8;
        for (i, x) in tms.iter().enumerate() {
            if *x != 0 {
                byte |= 1 << (i % 8);
            }
            if i % 8 == 7 {
                buf.push(byte);
                byte = 0;
            }
        }

        // Push the last byte for cases when we don't have a multiple of 8
        // transitions.
        if tms.len() % 8 != 0 {
            buf.push(byte);
        }

        let tdo_bytes = if tdo {
            vec![0xff; buf.len()]
        } else {
            vec![0; buf.len()]
        };

        self.tap_sequence(buf, tdo_bytes, tms.len());
    }

    fn read_data(&mut self, mut bits: usize) -> Vec<u8> {
        let bytes = (bits + 7) / 8;
        let buf = vec![0xff; bytes];

        bits %= 8;
        if bits == 0 {
            bits = 8;
        }
        self.read_write_data(&buf, bits as u8, false) 
    }

    fn write_data(&mut self, data: &[u8], bits: u8, pause_after: bool) {
        self.send_tdo(data, bits, pause_after);
    }

    fn read_write_data(&mut self, data: &[u8], bits: u8, pause_after: bool) -> Vec<u8> {
        let total_bits = (data.len()-1) * 8 + (bits as usize);

        // Anything already queued we don't care about
        let recv_bytes = self.recv_bytes;
        let send_bits = self.send_bits;

        self.send_tdo(data, bits, pause_after);
        self.flush_tap_sequence();

        // We actually care about these bytes, don't throw them away
        let bytes = self.recv_bytes - recv_bytes;
        self.recv_bytes = recv_bytes;
        let mut data = self.read_data(bytes).expect("read_write read");

        // We only want the last total_bits from this Vec, we may need to trim some leading bits
        let mut buf = vec![];
        bit_append(&mut buf, 0, &mut data, send_bits + total_bits, send_bits);
        buf
    }

    fn flush(&mut self) {
        self.read_data(0).expect("flush");
    }

    fn queue_read(&mut self, bits: usize) -> bool {
        let data = Cable::read_data(self, bits);
        self.read_queue.push(data);
        true
    }

    fn finish_read(&mut self, _bits: usize) -> Vec<u8> {
        self.read_queue.remove(0)
    }
}
