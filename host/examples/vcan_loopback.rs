//! End-to-end CANopen over a Linux `vcan0` virtual CAN bus — the whole stack on
//! a real bus (no hardware needed).
//!
//! A server thread runs a device [`Node`] that comes up **unconfigured**; the
//! main thread is a master that walks it through the full lifecycle:
//!
//! 1. **LSS** — assign the node its id (`0x10`) over the bus, then reset it.
//! 2. **SDO** — read/write objects (expedited and segmented).
//! 3. **NMT** — start the node into operational.
//! 4. **PDO** — drive an RPDO in and a SYNC-triggered TPDO out.
//! 5. **Block transfer** — stream a 50-byte block download and CRC-verify it.
//!
//! Set up the interface first (see `tools/vcan_setup.sh`), then:
//!
//! ```text
//! cargo run -p canopen-host --example vcan_loopback
//! ```

fn main() {
    #[cfg(target_os = "linux")]
    {
        if let Err(e) = linux::run() {
            eprintln!("vcan_loopback FAILED: {e}");
            std::process::exit(1);
        }
    }
    #[cfg(not(target_os = "linux"))]
    {
        eprintln!("This example requires Linux SocketCAN (vcan0); see tools/vcan_setup.sh.");
    }
}

#[cfg(target_os = "linux")]
mod linux {
    use std::error::Error;
    use std::sync::mpsc;
    use std::thread;
    use std::time::Duration;

    use canopen_host::transport::{Received, SocketCan};
    use canopen_rs::lss::{
        self, encode_configure_node_id, encode_store, encode_switch_global, LssAddress,
    };
    use canopen_rs::nmt::NMT_COMMAND_COB_ID;
    use canopen_rs::node::{Node, MAX_PDO_MAPPING};
    use canopen_rs::sdo::block::{
        self, decode_end, decode_sub_segment, encode_download_initiate, BlockReceiver, BlockWriter,
    };
    use canopen_rs::sync::SYNC_COB_ID;
    use canopen_rs::{
        Address, DataType, Entry, MappingEntry, NmtCommand, NodeId, ObjectDictionary, PdoMapping,
        TransmissionType, Value,
    };

    const IFACE: &str = "vcan0";
    // A private channel for the block-transfer demonstration (any unused ids).
    const BLOCK_REQ: u16 = 0x123;
    const BLOCK_RESP: u16 = 0x124;

    pub fn run() -> Result<(), Box<dyn Error>> {
        let (ready_tx, ready_rx) = mpsc::channel::<()>();
        thread::spawn(move || {
            if let Err(e) = serve(ready_tx) {
                eprintln!("server thread error: {e}");
            }
        });
        ready_rx.recv().map_err(|_| "server failed to start")?;

        let bus = SocketCan::open(IFACE)?;
        bus.set_read_timeout(Duration::from_secs(2))?;

        // --- 1. LSS: assign the unconfigured node its id (0x10), then reset. ---
        bus.send(lss::LSS_MASTER_COB_ID, &encode_switch_global(true))?;
        bus.send(lss::LSS_MASTER_COB_ID, &encode_configure_node_id(0x10))?;
        let cfg = recv_cob(&bus, lss::LSS_SLAVE_COB_ID)?;
        assert_eq!(&cfg.data()[..2], &[0x11, 0x00]); // configure success
        bus.send(lss::LSS_MASTER_COB_ID, &encode_store())?;
        recv_cob(&bus, lss::LSS_SLAVE_COB_ID)?; // store response
        bus.send(lss::LSS_MASTER_COB_ID, &encode_switch_global(false))?;
        bus.send_nmt(NmtCommand::ResetCommunication, NodeId::BROADCAST)?;
        let bootup = recv_cob(&bus, 0x700 + 0x10)?; // node boots on 0x710
        println!(
            "LSS: assigned node-id 0x10; node booted (bootup {:02X?})",
            bootup.data()
        );

        let node = NodeId::new(0x10)?;

        // --- 2. SDO: expedited and segmented. ---
        let device_type = bus.sdo_read(node, Address::new(0x1000, 0), DataType::Unsigned32)?;
        println!("SDO: read 0x1000 device type   = {device_type:?}");
        assert_eq!(device_type, Value::Unsigned32(0x0004_0192));
        bus.sdo_write(node, Address::new(0x1017, 0), Value::Unsigned16(2500))?;
        assert_eq!(
            bus.sdo_read(node, Address::new(0x1017, 0), DataType::Unsigned16)?,
            Value::Unsigned16(2500)
        );
        let big = Value::Unsigned64(0x0102_0304_0506_0708);
        bus.sdo_write(node, Address::new(0x2000, 0), big)?; // segmented
        assert_eq!(
            bus.sdo_read(node, Address::new(0x2000, 0), DataType::Unsigned64)?,
            big
        );
        println!("SDO: expedited + segmented read/write OK");

        // --- 3+4. NMT start, then RPDO in -> SYNC -> TPDO out. ---
        bus.send_nmt(NmtCommand::StartRemoteNode, node)?;
        bus.send(0x200 + 0x10, &[0xCD, 0xAB])?; // RPDO1 -> 0x6000/1 = 0xABCD
        bus.send(SYNC_COB_ID, &[])?;
        let tpdo = recv_cob(&bus, 0x180 + 0x10)?;
        assert_eq!(tpdo.data(), &[0xCD, 0xAB]);
        println!(
            "NMT+PDO: RPDO in 0xABCD -> SYNC -> TPDO out {:02X?}",
            tpdo.data()
        );

        // --- 5. Block transfer: stream 50 bytes, server CRC-verifies. ---
        let payload: [u8; 50] = core::array::from_fn(|i| i as u8);
        bus.send(
            BLOCK_REQ,
            &encode_download_initiate(Address::new(0x3000, 0), Some(payload.len() as u32), true),
        )?;
        let mut writer = BlockWriter::new(&payload, block::MAX_BLKSIZE);
        while let Some(segment) = writer.next_segment() {
            bus.send(BLOCK_REQ, &segment)?;
        }
        bus.send(BLOCK_REQ, &writer.end_frame(true))?;
        let confirm = recv_cob(&bus, BLOCK_RESP)?;
        assert_eq!(confirm.data(), &[1], "server CRC-verified the block");
        println!(
            "BLOCK: downloaded {} bytes -> server CRC-verified",
            payload.len()
        );

        println!("\nvcan0 loopback OK — LSS, SDO, NMT, PDO, and block transfer all round-tripped.");
        Ok(())
    }

    /// Receive frames until one arrives on `cob_id`.
    fn recv_cob(bus: &SocketCan, cob_id: u16) -> Result<Received, Box<dyn Error>> {
        loop {
            let frame = bus.recv()?;
            if frame.cob_id == cob_id {
                return Ok(frame);
            }
        }
    }

    /// The device node: unconfigured until LSS assigns an id, then a full node.
    fn serve(ready: mpsc::Sender<()>) -> Result<(), Box<dyn Error>> {
        let bus = SocketCan::open(IFACE)?;
        bus.set_read_timeout(Duration::from_secs(3))?;

        let mut od = ObjectDictionary::<8>::new();
        od.insert(
            Address::new(0x1000, 0),
            Entry::constant(Value::Unsigned32(0x0004_0192)),
        )?;
        od.insert(Address::new(0x1017, 0), Entry::rw(Value::Unsigned16(1000)))?;
        od.insert(Address::new(0x2000, 0), Entry::rw(Value::Unsigned64(0)))?;
        od.insert(Address::new(0x6000, 1), Entry::rw(Value::Unsigned16(0)))?;

        // Provisional id 1 while unconfigured; LSS assigns the real one.
        let mut node = Node::new(NodeId::new(1)?, od);
        node.enable_lss(LssAddress {
            vendor_id: 0x1F,
            product_code: 0x2A,
            revision_number: 1,
            serial_number: 0x99,
        });
        ready.send(()).map_err(|_| "client went away")?;

        let mut block_rx = BlockReceiver::<256>::new();
        let mut booted = false;

        while let Ok(frame) = bus.recv() {
            if frame.cob_id == BLOCK_REQ {
                handle_block(&bus, &mut block_rx, &frame)?;
                continue;
            }
            if let Some(tx) = node.on_frame(frame.cob_id, frame.data()) {
                bus.send(tx.cob_id, tx.data())?;
            }
            // The reset that follows LSS configuration: adopt the id and boot.
            if !booted && frame.cob_id == NMT_COMMAND_COB_ID {
                node.apply_lss_node_id();
                let id = node.node_id().raw() as u16;
                node.add_rpdo(0x200 + id, single_map(0x6000, 1, 16))?;
                node.add_tpdo(
                    0x180 + id,
                    single_map(0x6000, 1, 16),
                    TransmissionType::SynchronousAcyclic,
                )?;
                let boot = node.boot();
                bus.send(boot.cob_id, boot.data())?;
                booted = true;
            }
            if frame.cob_id == SYNC_COB_ID {
                for tx in node.sync_tpdos() {
                    bus.send(tx.cob_id, tx.data())?;
                }
            }
        }
        Ok(())
    }

    /// Reassemble a streamed block download and CRC-verify it on the end frame.
    fn handle_block(
        bus: &SocketCan,
        rx: &mut BlockReceiver<256>,
        frame: &Received,
    ) -> Result<(), Box<dyn Error>> {
        let p = pad8(frame.data());
        let cmd = p[0];
        if cmd & 0xE0 == 0xC0 && cmd & 0x01 == 0 {
            *rx = BlockReceiver::new(); // initiate: start fresh
        } else if cmd & 0xE0 == 0xC0 && cmd & 0x01 == 1 {
            let (unused, crc) = decode_end(&p)?;
            let ok = rx.finish(unused, crc, true).is_ok();
            bus.send(BLOCK_RESP, &[ok as u8])?; // one confirmation frame
        } else {
            rx.push(&decode_sub_segment(&p))?; // a sub-block segment
        }
        Ok(())
    }

    fn pad8(data: &[u8]) -> [u8; 8] {
        let mut p = [0u8; 8];
        let n = data.len().min(8);
        p[..n].copy_from_slice(&data[..n]);
        p
    }

    fn single_map(index: u16, sub: u8, bits: u8) -> PdoMapping<MAX_PDO_MAPPING> {
        let mut m = PdoMapping::new();
        m.push(MappingEntry::new(index, sub, bits))
            .expect("one entry fits");
        m
    }
}
