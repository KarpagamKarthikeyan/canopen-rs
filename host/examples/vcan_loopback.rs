//! End-to-end SDO over a Linux `vcan0` virtual CAN bus — the first time the
//! SocketCAN transport actually executes on a bus (no hardware needed).
//!
//! A server thread exposes an object dictionary on `vcan0`; the main thread is
//! a client that reads and writes it, exercising both expedited and segmented
//! transfers. Because SocketCAN delivers every frame to all sockets bound to
//! the interface, the two ends simply share `vcan0`.
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

    use canopen_host::transport::SocketCan;
    use canopen_rs::sdo::SdoServer;
    use canopen_rs::{Address, DataType, Entry, NodeId, ObjectDictionary, Value};

    const IFACE: &str = "vcan0";

    pub fn run() -> Result<(), Box<dyn Error>> {
        let node = NodeId::new(0x10)?;

        // --- Server node: serve an object dictionary on vcan0 in a thread. ---
        let (ready_tx, ready_rx) = mpsc::channel::<()>();
        thread::spawn(move || {
            if let Err(e) = serve(node, ready_tx) {
                eprintln!("server thread error: {e}");
            }
        });
        // Wait until the server socket is open and listening.
        ready_rx.recv().map_err(|_| "server failed to start")?;

        // --- Client: talk to the node over vcan0. ---
        let bus = SocketCan::open(IFACE)?;
        bus.set_read_timeout(Duration::from_secs(2))?;

        // Expedited read (4-byte object).
        let device_type = bus.sdo_read(node, Address::new(0x1000, 0), DataType::Unsigned32)?;
        println!("read  0x1000 device type      = {device_type:?}");
        assert_eq!(device_type, Value::Unsigned32(0x0004_0192));

        // Expedited write then read-back (2-byte object).
        bus.sdo_write(node, Address::new(0x1017, 0), Value::Unsigned16(2500))?;
        let heartbeat = bus.sdo_read(node, Address::new(0x1017, 0), DataType::Unsigned16)?;
        println!("write 0x1017 heartbeat -> read = {heartbeat:?}");
        assert_eq!(heartbeat, Value::Unsigned16(2500));

        // 8-byte object forces SEGMENTED transfer over the real bus.
        let big = Value::Unsigned64(0x0102_0304_0506_0708);
        bus.sdo_write(node, Address::new(0x2000, 0), big)?;
        let back = bus.sdo_read(node, Address::new(0x2000, 0), DataType::Unsigned64)?;
        println!("write 0x2000 (segmented) -> read = {back:?}");
        assert_eq!(back, big);

        println!("\nvcan0 loopback OK — expedited and segmented SDO round-trips succeeded.");
        Ok(())
    }

    /// A minimal device node: build an OD and answer SDO requests addressed to
    /// this node until the bus goes quiet.
    fn serve(node: NodeId, ready: mpsc::Sender<()>) -> Result<(), Box<dyn Error>> {
        let bus = SocketCan::open(IFACE)?;
        bus.set_read_timeout(Duration::from_secs(3))?;

        let mut od = ObjectDictionary::<8>::new();
        od.insert(
            Address::new(0x1000, 0),
            Entry::constant(Value::Unsigned32(0x0004_0192)),
        )?;
        od.insert(Address::new(0x1017, 0), Entry::rw(Value::Unsigned16(1000)))?;
        od.insert(Address::new(0x2000, 0), Entry::rw(Value::Unsigned64(0)))?;

        let mut server = SdoServer::new(node);
        ready.send(()).map_err(|_| "client went away")?;

        // Serve until a read times out (the client is done and the bus is idle).
        while let Ok(frame) = bus.recv() {
            if frame.cob_id != server.request_cob_id() {
                continue;
            }
            if let Some(response) = server.handle(&mut od, frame.payload()) {
                bus.send(server.response_cob_id(), &response)?;
            }
        }
        Ok(())
    }
}
