//! Print every modeled EDS/DCF object as plain text.
//!
//! Run:
//! ```text
//! cargo run -p canopen-host --example dump_eds path/to/device.eds
//! ```
//!
//! The output includes address (`index:subindex`), parameter name, data type, and
//! default/configured value.

use canopen_host::eds::Eds;

fn main() {
    let path = std::env::args().nth(1).unwrap_or_else(|| {
        eprintln!("Usage: dump_eds <path>");
        std::process::exit(1);
    });

    let eds = match Eds::from_file(&path) {
        Ok(eds) => eds,
        Err(e) => {
            eprintln!("Failed to load '{path}': {e}");
            std::process::exit(2);
        }
    };

    if let Some(vendor) = eds.vendor_name.as_deref() {
        println!("Vendor: {vendor}");
    }
    if let Some(product) = eds.product_name.as_deref() {
        println!("Product: {product}");
    }
    if let Some(node_id) = eds.node_id {
        println!("NodeID: 0x{node_id:02X}");
    }

    println!("Index:Sub Index | Name | DataType | Default");
    for object in eds.objects {
        println!(
            "0x{:04X}:{:02X} | {} | {:?} | {:?}",
            object.address.index,
            object.address.subindex,
            object.parameter_name,
            object.data_type,
            object.default_value
        );
    }
}
