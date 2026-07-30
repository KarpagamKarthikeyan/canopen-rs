//! Process Data Object (PDO) — real-time process data (CiA 301 §7.2.2).
//!
//! PDOs carry mapped process data with no protocol overhead: up to eight
//! bytes of application data per frame, laid out by the PDO mapping
//! parameters. Receive PDOs (RPDO) consume data into the OD; transmit PDOs
//! (TPDO) publish data from it.

// TODO: the PDO mapping model (communication + mapping parameter objects)
// and RPDO/TPDO encode/decode.
