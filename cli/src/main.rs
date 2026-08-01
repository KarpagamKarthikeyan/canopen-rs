//! `canopen` — a command-line tool for CANopen buses.
//!
//! Inspect EDS files anywhere; read/write objects over SDO, send NMT commands,
//! and monitor traffic on a Linux SocketCAN interface.

// On non-Linux the bus subcommands are stubbed out, so their argument-parsing
// helpers are unused there — this is a reduced build with only `canopen eds`.
#![cfg_attr(not(target_os = "linux"), allow(dead_code))]

use std::path::PathBuf;

use anyhow::{bail, Context, Result};
use clap::{Parser, Subcommand};

use canopen_rs::{Address, DataType, NmtCommand, Value};

#[derive(Parser)]
#[command(
    name = "canopen",
    version,
    about = "Command-line tool for CANopen buses"
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Parse an EDS/DCF file and list its objects.
    Eds {
        /// Path to the `.eds` / `.dcf` file.
        file: PathBuf,
    },
    /// Read an object from a node over SDO (Linux SocketCAN).
    Read {
        /// CAN interface, e.g. `can0` or `vcan0`.
        interface: String,
        /// Target node id (1..=127).
        node: u8,
        /// Object as `INDEX[:SUB]` in hex, e.g. `1017` or `1018:1`.
        object: String,
        /// Data type: u8|u16|u32|u64|i8|i16|i32|i64|f32|f64|bool.
        datatype: String,
    },
    /// Write an object to a node over SDO (Linux SocketCAN).
    Write {
        /// CAN interface, e.g. `can0` or `vcan0`.
        interface: String,
        /// Target node id (1..=127).
        node: u8,
        /// Object as `INDEX[:SUB]` in hex.
        object: String,
        /// Data type: u8|u16|u32|u64|i8|i16|i32|i64|f32|f64|bool.
        datatype: String,
        /// Value: decimal or `0x` hex for integers; `true`/`false` for bool.
        value: String,
    },
    /// Send an NMT node-control command (Linux SocketCAN).
    Nmt {
        /// CAN interface, e.g. `can0` or `vcan0`.
        interface: String,
        /// Command: start|stop|preop|reset|reset-comm.
        command: String,
        /// Target node id, or 0 for all nodes (default).
        #[arg(default_value_t = 0)]
        node: u8,
    },
    /// Print received frames until interrupted (Linux SocketCAN).
    Monitor {
        /// CAN interface, e.g. `can0` or `vcan0`.
        interface: String,
    },
}

fn main() -> Result<()> {
    match Cli::parse().command {
        Command::Eds { file } => cmd_eds(&file),
        Command::Read {
            interface,
            node,
            object,
            datatype,
        } => cmd_read(&interface, node, &object, &datatype),
        Command::Write {
            interface,
            node,
            object,
            datatype,
            value,
        } => cmd_write(&interface, node, &object, &datatype, &value),
        Command::Nmt {
            interface,
            command,
            node,
        } => cmd_nmt(&interface, &command, node),
        Command::Monitor { interface } => cmd_monitor(&interface),
    }
}

// --- EDS inspection (cross-platform) ---------------------------------------

fn cmd_eds(file: &PathBuf) -> Result<()> {
    use canopen_host::eds::Eds;

    let eds = Eds::from_file(file).with_context(|| format!("reading {}", file.display()))?;
    println!(
        "vendor:  {}",
        eds.vendor_name.as_deref().unwrap_or("(none)")
    );
    println!(
        "product: {}",
        eds.product_name.as_deref().unwrap_or("(none)")
    );
    if let Some(id) = eds.node_id {
        println!("node id: {id:#04x}");
    }
    println!("{} objects:", eds.objects.len());
    for obj in &eds.objects {
        println!(
            "  {:#06x}:{:02x}  {:<10} {:<6} {:<24} {}",
            obj.address.index,
            obj.address.subindex,
            format!("{:?}", obj.data_type),
            format!("{:?}", obj.access),
            format!("{:?}", obj.default_value),
            obj.parameter_name,
        );
    }
    Ok(())
}

// --- Argument parsing (cross-platform, unit-tested) ------------------------
// Consumed by the Linux `bus` module and the tests; "unused" on non-Linux
// builds (which only have `canopen eds`) is expected — see the crate-level
// allow above.

/// Parse `INDEX[:SUB]` (hex) into an [`Address`]; subindex defaults to 0.
fn parse_object(s: &str) -> Result<Address> {
    let (index, sub) = s.split_once(':').unwrap_or((s, "0"));
    let index = u16::from_str_radix(index.trim_start_matches("0x"), 16)
        .with_context(|| format!("invalid object index '{index}' (expected hex)"))?;
    let subindex = u8::from_str_radix(sub.trim_start_matches("0x"), 16)
        .with_context(|| format!("invalid subindex '{sub}' (expected hex)"))?;
    Ok(Address::new(index, subindex))
}

fn parse_datatype(s: &str) -> Result<DataType> {
    Ok(match s.to_ascii_lowercase().as_str() {
        "u8" => DataType::Unsigned8,
        "u16" => DataType::Unsigned16,
        "u32" => DataType::Unsigned32,
        "u64" => DataType::Unsigned64,
        "i8" => DataType::Integer8,
        "i16" => DataType::Integer16,
        "i32" => DataType::Integer32,
        "i64" => DataType::Integer64,
        "f32" => DataType::Real32,
        "f64" => DataType::Real64,
        "bool" => DataType::Boolean,
        other => {
            bail!("unknown data type '{other}' (use u8|u16|u32|u64|i8|i16|i32|i64|f32|f64|bool)")
        }
    })
}

/// Parse a signed integer literal — decimal, or `0x`/`0X` hex.
fn parse_int(s: &str) -> Result<i128> {
    let s = s.trim();
    match s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        Some(hex) => i128::from_str_radix(hex, 16).with_context(|| format!("invalid hex '{s}'")),
        None => s
            .parse::<i128>()
            .with_context(|| format!("invalid integer '{s}'")),
    }
}

/// Parse `value` as `dt`.
fn parse_value(dt: DataType, value: &str) -> Result<Value> {
    Ok(match dt {
        DataType::Boolean => Value::Boolean(matches!(
            value.trim().to_ascii_lowercase().as_str(),
            "true" | "1"
        )),
        DataType::Real32 => Value::Real32(value.trim().parse().context("invalid f32")?),
        DataType::Real64 => Value::Real64(value.trim().parse().context("invalid f64")?),
        DataType::Unsigned8 => Value::Unsigned8(parse_int(value)? as u8),
        DataType::Unsigned16 => Value::Unsigned16(parse_int(value)? as u16),
        DataType::Unsigned32 => Value::Unsigned32(parse_int(value)? as u32),
        DataType::Unsigned64 => Value::Unsigned64(parse_int(value)? as u64),
        DataType::Integer8 => Value::Integer8(parse_int(value)? as i8),
        DataType::Integer16 => Value::Integer16(parse_int(value)? as i16),
        DataType::Integer32 => Value::Integer32(parse_int(value)? as i32),
        DataType::Integer64 => Value::Integer64(parse_int(value)? as i64),
        _ => bail!("unsupported data type for writing"),
    })
}

fn parse_nmt_command(s: &str) -> Result<NmtCommand> {
    Ok(match s.to_ascii_lowercase().as_str() {
        "start" => NmtCommand::StartRemoteNode,
        "stop" => NmtCommand::StopRemoteNode,
        "preop" | "pre-op" | "pre-operational" => NmtCommand::EnterPreOperational,
        "reset" | "reset-node" => NmtCommand::ResetNode,
        "reset-comm" | "reset-communication" => NmtCommand::ResetCommunication,
        other => bail!("unknown NMT command '{other}' (use start|stop|preop|reset|reset-comm)"),
    })
}

// --- Bus commands (Linux SocketCAN) ----------------------------------------

#[cfg(target_os = "linux")]
mod bus {
    use super::*;
    use std::time::Duration;

    use canopen_host::transport::SocketCan;
    use canopen_rs::types::NodeId;

    fn node_id(node: u8) -> Result<NodeId> {
        NodeId::new(node).map_err(|_| anyhow::anyhow!("node id {node} out of range (1..=127)"))
    }

    fn open(interface: &str) -> Result<SocketCan> {
        let bus = SocketCan::open(interface)
            .with_context(|| format!("opening interface '{interface}'"))?;
        bus.set_read_timeout(Duration::from_secs(1))?;
        Ok(bus)
    }

    pub fn read(interface: &str, node: u8, object: &str, datatype: &str) -> Result<()> {
        let bus = open(interface)?;
        let value = bus
            .sdo_read(
                node_id(node)?,
                parse_object(object)?,
                parse_datatype(datatype)?,
            )
            .map_err(|e| anyhow::anyhow!("SDO read failed: {e}"))?;
        println!("{value:?}");
        Ok(())
    }

    pub fn write(
        interface: &str,
        node: u8,
        object: &str,
        datatype: &str,
        value: &str,
    ) -> Result<()> {
        let bus = open(interface)?;
        let dt = parse_datatype(datatype)?;
        bus.sdo_write(
            node_id(node)?,
            parse_object(object)?,
            parse_value(dt, value)?,
        )
        .map_err(|e| anyhow::anyhow!("SDO write failed: {e}"))?;
        println!("ok");
        Ok(())
    }

    pub fn nmt(interface: &str, command: &str, node: u8) -> Result<()> {
        let bus = open(interface)?;
        let target = if node == 0 {
            NodeId::BROADCAST
        } else {
            node_id(node)?
        };
        bus.send_nmt(parse_nmt_command(command)?, target)?;
        println!(
            "sent {command} to {}",
            if node == 0 {
                "all nodes".into()
            } else {
                format!("node {node:#04x}")
            }
        );
        Ok(())
    }

    pub fn monitor(interface: &str) -> Result<()> {
        let bus = SocketCan::open(interface)
            .with_context(|| format!("opening interface '{interface}'"))?;
        println!("monitoring {interface} (Ctrl-C to stop)…");
        loop {
            let frame = bus.recv().context("receiving frame")?;
            let hex: String = frame
                .data()
                .iter()
                .map(|b| format!("{b:02X}"))
                .collect::<Vec<_>>()
                .join(" ");
            println!(
                "{:>3X}  [{:<23}] {}",
                frame.cob_id,
                hex,
                classify(frame.cob_id)
            );
        }
    }

    /// A short label for a COB-ID's function.
    fn classify(cob_id: u16) -> &'static str {
        match cob_id {
            0x000 => "NMT",
            0x080 => "SYNC",
            0x081..=0x0FF => "EMCY",
            0x100 => "TIME",
            0x180..=0x57F => "PDO",
            0x580..=0x5FF => "SDO tx",
            0x600..=0x67F => "SDO rx",
            0x700..=0x77F => "heartbeat",
            0x7E4 | 0x7E5 => "LSS",
            _ => "",
        }
    }
}

#[cfg(target_os = "linux")]
fn cmd_read(interface: &str, node: u8, object: &str, datatype: &str) -> Result<()> {
    bus::read(interface, node, object, datatype)
}
#[cfg(target_os = "linux")]
fn cmd_write(interface: &str, node: u8, object: &str, datatype: &str, value: &str) -> Result<()> {
    bus::write(interface, node, object, datatype, value)
}
#[cfg(target_os = "linux")]
fn cmd_nmt(interface: &str, command: &str, node: u8) -> Result<()> {
    bus::nmt(interface, command, node)
}
#[cfg(target_os = "linux")]
fn cmd_monitor(interface: &str) -> Result<()> {
    bus::monitor(interface)
}

#[cfg(not(target_os = "linux"))]
const NOT_LINUX: &str = "bus commands require Linux SocketCAN; only `canopen eds` runs here";
#[cfg(not(target_os = "linux"))]
fn cmd_read(_: &str, _: u8, _: &str, _: &str) -> Result<()> {
    bail!(NOT_LINUX)
}
#[cfg(not(target_os = "linux"))]
fn cmd_write(_: &str, _: u8, _: &str, _: &str, _: &str) -> Result<()> {
    bail!(NOT_LINUX)
}
#[cfg(not(target_os = "linux"))]
fn cmd_nmt(_: &str, _: &str, _: u8) -> Result<()> {
    bail!(NOT_LINUX)
}
#[cfg(not(target_os = "linux"))]
fn cmd_monitor(_: &str) -> Result<()> {
    bail!(NOT_LINUX)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_object_addresses() {
        assert_eq!(parse_object("1017").unwrap(), Address::new(0x1017, 0));
        assert_eq!(parse_object("1018:1").unwrap(), Address::new(0x1018, 1));
        assert_eq!(parse_object("0x6000:0x2").unwrap(), Address::new(0x6000, 2));
        assert!(parse_object("xyz").is_err());
    }

    #[test]
    fn parses_data_types() {
        assert_eq!(parse_datatype("u32").unwrap(), DataType::Unsigned32);
        assert_eq!(parse_datatype("I16").unwrap(), DataType::Integer16);
        assert_eq!(parse_datatype("bool").unwrap(), DataType::Boolean);
        assert!(parse_datatype("u128").is_err());
    }

    #[test]
    fn parses_values() {
        assert_eq!(
            parse_value(DataType::Unsigned16, "1000").unwrap(),
            Value::Unsigned16(1000)
        );
        assert_eq!(
            parse_value(DataType::Unsigned32, "0x1F4").unwrap(),
            Value::Unsigned32(0x1F4)
        );
        assert_eq!(
            parse_value(DataType::Integer8, "-5").unwrap(),
            Value::Integer8(-5)
        );
        assert_eq!(
            parse_value(DataType::Boolean, "true").unwrap(),
            Value::Boolean(true)
        );
        assert_eq!(
            parse_value(DataType::Real32, "1.5").unwrap(),
            Value::Real32(1.5)
        );
        assert!(parse_value(DataType::Unsigned8, "notnum").is_err());
    }

    #[test]
    fn parses_nmt_commands() {
        assert_eq!(
            parse_nmt_command("start").unwrap(),
            NmtCommand::StartRemoteNode
        );
        assert_eq!(
            parse_nmt_command("reset-comm").unwrap(),
            NmtCommand::ResetCommunication
        );
        assert!(parse_nmt_command("frobnicate").is_err());
    }
}
