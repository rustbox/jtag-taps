//! Implement the `Cable` trait for "jlink" compatible hardware adapters
use crate::cable::Cable;

use std::time::Duration;

use rusb::{DeviceHandle, Direction, GlobalContext};
use rusb::constants::*;

pub struct JLink {
    device: DeviceHandle<GlobalContext>,
    read_endpoint: u8,
    write_endpoint: u8,
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

                println!("found it");

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

    fn send_command(&mut self, cmd: u8, mut data: Vec<u8>) -> Result<(),rusb::Error> {
        data.insert(0, cmd);
        let len = self.device.write_bulk(self.write_endpoint, &data, Duration::from_millis(100))?;
        assert_eq!(len, data.len());
        Ok(())
    }

    fn read_data(&mut self, len: usize) -> Result<Vec<u8>, rusb::Error> {
        let mut data = vec![0; len];
        let len = self.device.read_bulk(self.read_endpoint, &mut data, Duration::from_millis(100))?;
        assert_eq!(len, data.len());
        Ok(data)
    }

    pub fn get_status(&mut self) -> Vec<u8> {
        self.send_command(0x7, vec![]).expect("status");
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
        self.send_command(0x5, buf).expect("set clock");
    }

    pub fn set_interface(&mut self, intf: u8) {
        let buf = vec![intf];
        self.send_command(0xc7, buf).expect("set interface");
        let _ = self.read_data(4).expect("set interface response");
    }

    pub fn assert_srst(&mut self) {
        self.send_command(0xdc, vec![]).expect("assert srst");
    }

    pub fn deassert_srst(&mut self) {
        self.send_command(0xdd, vec![]).expect("deassert srst");
    }

    pub fn assert_trst(&mut self) {
        self.send_command(0xde, vec![]).expect("assert trst");
    }

    pub fn deassert_trst(&mut self) {
        self.send_command(0xdf, vec![]).expect("deassert trst");
    }

    fn tap_sequence(&mut self, mut tms: Vec<u8>, mut tdo: Vec<u8>, bits: usize) -> Result<Vec<u8>, rusb::Error> {
        assert_eq!(tms.len(), tdo.len());
        assert!(tms.len() < 390);
        let bytes = tms.len();
        let mut cmdbuf = vec![(bits & 0xff) as u8, ((bits >> 8) & 0xff) as u8];
        cmdbuf.append(&mut tms);
        cmdbuf.append(&mut tdo);

        self.send_command(0xcd, cmdbuf)?;
        self.read_data(bytes)
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

        self.tap_sequence(buf, tdo_bytes, tms.len()).expect("change mode");
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
        self.read_write_data(data, bits, pause_after);
    }

    fn read_write_data(&mut self, data: &[u8], bits: u8, pause_after: bool) -> Vec<u8> {
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

        self.tap_sequence(tms, data, total_bits).expect("read write data")
    }
}
