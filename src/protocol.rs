#![allow(dead_code)]
use super::BUFFER_LEN;
use core::{
    fmt,
    fmt::{Debug, Write},
};
use embedded_nal_async::{IpAddr, Ipv4Addr, SocketAddr};
use heapless::String;

#[derive(Debug)]
#[doc(hidden)]
pub struct ResolverAddresses {
    pub(crate) resolver1: Ipv4Addr,
    pub(crate) resolver2: Option<Ipv4Addr>,
}

#[cfg(feature = "defmt")]
impl defmt::Format for ResolverAddresses {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(
            f,
            "resolver1: {}, resolver2: {}",
            self.resolver1.octets(),
            self.resolver2.map(|o| o.octets())
        );
    }
}

/// Type of socket connection.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum ConnectionType {
    TCP,
    UDP,
}

/// Mode of the Wi-Fi stack
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum WiFiMode {
    /// Station mode, aka client
    Station,
    /// Access point mode
    SoftAccessPoint,
    /// Access point + station mode
    SoftAccessPointAndStation,
}

/// Commands to be sent to the ESP board.
#[derive(Debug)]
pub(crate) enum Command<'a> {
    QueryFirmwareInfo,
    SetMode(WiFiMode),
    JoinAp { ssid: &'a str, password: &'a str },
    QueryIpAddress,
    StartConnection(usize, ConnectionType, SocketAddr),
    CloseConnection(usize),
    Send { link_id: usize, len: usize },
    Receive { link_id: usize, len: usize },
    QueryDnsResolvers,
    SetDnsResolvers(ResolverAddresses),
    GetHostByName { hostname: &'a str },
}

#[cfg(feature = "defmt")]
impl<'a> defmt::Format for Command<'a> {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(f, "{}", &self.as_bytes());
    }
}

impl<'a> Command<'a> {
    pub(crate) fn as_bytes(&self) -> String<256> {
        match self {
            Command::QueryFirmwareInfo => String::from("AT+GMR"),
            Command::QueryIpAddress => String::from("AT+CIPSTA_CUR?"),
            Command::SetMode(mode) => match mode {
                WiFiMode::Station => String::from("AT+CWMODE_CUR=1"),
                WiFiMode::SoftAccessPoint => String::from("AT+CWMODE_CUR=2"),
                WiFiMode::SoftAccessPointAndStation => String::from("AT+CWMODE_CUR=3"),
            },
            Command::JoinAp { ssid, password } => {
                let mut s = String::from("AT+CWJAP_CUR=\"");
                s.push_str(ssid).unwrap();
                s.push_str("\",\"").unwrap();
                s.push_str(password).unwrap();
                s.push_str("\"").unwrap();
                s
            }
            Command::StartConnection(link_id, connection_type, socket_addr) => {
                let mut s = String::from("AT+CIPSTART=");
                write!(s, "{},", link_id).unwrap();
                match connection_type {
                    ConnectionType::TCP => {
                        write!(s, "\"TCP\"").unwrap();
                    }
                    ConnectionType::UDP => {
                        write!(s, "\"UDP\"").unwrap();
                    }
                }
                write!(s, ",").unwrap();
                match socket_addr.ip() {
                    IpAddr::V4(ip) => {
                        write!(s, "\"{}\",{}", ip, socket_addr.port()).unwrap();
                    }
                    IpAddr::V6(_) => panic!("IPv6 not supported"),
                }
                s as String<256>
            }
            Command::CloseConnection(link_id) => {
                let mut s = String::from("AT+CIPCLOSE=");
                write!(s, "{}", link_id).unwrap();
                s
            }
            Command::Send { link_id, len } => {
                let mut s = String::from("AT+CIPSEND=");
                write!(s, "{},{}", link_id, len).unwrap();
                s
            }
            Command::Receive { link_id, len } => {
                let mut s = String::from("AT+CIPRECVDATA=");
                write!(s, "{},{}", link_id, len).unwrap();
                s
            }
            Command::QueryDnsResolvers => String::from("AT+CIPDNS_CUR?"),
            Command::SetDnsResolvers(addr) => {
                let mut s = String::from("AT+CIPDNS_CUR=1,");
                write!(s, "\"{}\"", addr.resolver1).unwrap();
                if let Some(resolver2) = addr.resolver2 {
                    write!(s, ",\"{}\"", resolver2).unwrap()
                }
                s
            }
            Command::GetHostByName { hostname } => {
                let mut s = String::from("AT+CIPDOMAIN=");
                write!(s, "\"{}\"", hostname).unwrap();
                s
            }
        }
    }
}

/// Responses (including unsolicited) which may be parsed from the board.
#[allow(clippy::large_enum_variant)]
#[doc(hidden)]
pub enum Response {
    None,
    Ok,
    Error,
    FirmwareInfo(FirmwareInfo),
    ReadyForData,
    ReceivedDataToSend(usize),
    SendOk,
    SendFail,
    DataAvailable { link_id: usize, len: usize },
    DataReceived([u8; BUFFER_LEN], usize),
    WifiConnected,
    WifiConnectionFailure(WifiConnectionFailure),
    WifiDisconnect,
    GotIp,
    IpAddresses(IpAddresses),
    Connect(usize),
    Closed(usize),
    Resolvers(ResolverAddresses),
    IpAddress(Ipv4Addr),
    DnsFail,
    UnlinkFail,
}

#[cfg(feature = "defmt")]
impl defmt::Format for Response {
    fn format(&self, f: defmt::Formatter<'_>) {
        match self {
            Response::None => defmt::write!(f, "None"),
            Response::Ok => defmt::write!(f, "Ok"),
            Response::Error => defmt::write!(f, "Error"),
            Response::FirmwareInfo(v) => defmt::write!(f, "FirmwareInfo: {}", v),
            Response::ReadyForData => defmt::write!(f, "ReadyForData"),
            Response::ReceivedDataToSend(len) => {
                defmt::write!(f, "ReceivedDataToSend {}", len)
            }
            Response::SendOk => defmt::write!(f, "SendOk"),
            Response::SendFail => defmt::write!(f, "SendFail"),
            Response::DataAvailable { link_id, len } => {
                defmt::write!(f, "DataAvailable link_id({}), len({})", link_id, len)
            }
            //Response::DataReceived(d, l) => dump_data("DataReceived", d, *l, f),
            Response::DataReceived(_, _) => defmt::write!(f, "DataReceived"),
            Response::WifiConnected => defmt::write!(f, "WifiConnected"),
            Response::WifiConnectionFailure(v) => defmt::write!(f, "WifiConnectionFailure {}", v),
            Response::WifiDisconnect => defmt::write!(f, "WifiDisconnect"),
            Response::GotIp => defmt::write!(f, "GotIp"),
            Response::IpAddresses(v) => defmt::write!(f, "IpAddresses: {}", v),
            Response::Connect(v) => defmt::write!(f, "Connect {}", v),
            Response::Closed(v) => defmt::write!(f, "Closed {}", v),
            Response::IpAddress(v) => {
                let ip = v.octets();
                defmt::write!(f, "IpAddress {}.{}.{}.{}", ip[0], ip[1], ip[2], ip[3])
            }
            Response::Resolvers(v) => defmt::write!(f, "Resolvers {}", v),
            Response::DnsFail => defmt::write!(f, "DNS Fail"),
            Response::UnlinkFail => defmt::write!(f, "UnlinkFail"),
        }
    }
}

impl Debug for Response {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Response::None => f.write_str("None"),
            Response::Ok => f.write_str("Ok"),
            Response::Error => f.write_str("Error"),
            Response::FirmwareInfo(v) => f.debug_tuple("FirmwareInfo").field(v).finish(),
            Response::ReadyForData => f.write_str("ReadyForData"),
            Response::ReceivedDataToSend(len) => {
                f.debug_tuple("ReceivedDataToSend").field(len).finish()
            }
            Response::SendOk => f.write_str("SendOk"),
            Response::SendFail => f.write_str("SendFail"),
            Response::DataAvailable { link_id, len } => f
                .debug_struct("DataAvailable")
                .field("link_id", link_id)
                .field("len", len)
                .finish(),
            //Response::DataReceived(d, l) => dump_data("DataReceived", d, *l, f),
            Response::DataReceived(_, _) => f.write_str("DataReceived"),
            Response::WifiConnected => f.write_str("WifiConnected"),
            Response::WifiConnectionFailure(v) => {
                f.debug_tuple("WifiConnectionFailure").field(v).finish()
            }
            Response::WifiDisconnect => f.write_str("WifiDisconnect"),
            Response::GotIp => f.write_str("GotIp"),
            Response::IpAddresses(v) => f.debug_tuple("IpAddresses").field(v).finish(),
            Response::Connect(v) => f.debug_tuple("Connect").field(v).finish(),
            Response::Closed(v) => f.debug_tuple("Closed").field(v).finish(),
            Response::IpAddress(v) => f.debug_tuple("IpAddress").field(v).finish(),
            Response::Resolvers(v) => f.debug_tuple("Resolvers").field(v).finish(),
            Response::DnsFail => f.write_str("DNS Fail"),
            Response::UnlinkFail => f.write_str("UnlinkFail"),
        }
    }
}

/// IP addresses for the board, including its own address, netmask and gateway.
#[derive(Debug)]
#[doc(hidden)]
pub struct IpAddresses {
    pub(crate) ip: Ipv4Addr,
    pub(crate) gateway: Ipv4Addr,
    pub(crate) netmask: Ipv4Addr,
}

#[cfg(feature = "defmt")]
impl defmt::Format for IpAddresses {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(
            f,
            "ip: {}, gateway: {}, netmask: {}",
            self.ip.octets(),
            self.gateway.octets(),
            self.netmask.octets(),
        );
    }
}

/// Version information for the ESP board.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[doc(hidden)]
pub struct FirmwareInfo {
    pub(crate) major: u8,
    pub(crate) minor: u8,
    pub(crate) patch: u8,
    pub(crate) build: u8,
}

/// Reasons for Wifi access-point join failures.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[doc(hidden)]
pub enum WifiConnectionFailure {
    Timeout,
    WrongPassword,
    CannotFindTargetAp,
    ConnectionFailed,
}

impl From<u8> for WifiConnectionFailure {
    fn from(code: u8) -> Self {
        match code {
            1 => WifiConnectionFailure::Timeout,
            2 => WifiConnectionFailure::WrongPassword,
            3 => WifiConnectionFailure::CannotFindTargetAp,
            _ => WifiConnectionFailure::ConnectionFailed,
        }
    }
}

/// Dump some data, which is stored in a buffer with a length indicator.
///
/// The output will contain the field name, the data as string (only 7bits) and the raw bytes
/// in hex encoding.
#[allow(dead_code)]
fn dump_data(name: &str, data: &[u8], len: usize, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    let data = &data[0..len];

    f.write_str(name)?;
    f.write_char('(')?;

    f.write_fmt(format_args!("{}; '", len))?;

    for d in data {
        if *d == 0 {
            f.write_str("\\0")?;
        } else if *d <= 0x7F {
            f.write_char(*d as char)?;
        } else {
            f.write_char('\u{FFFD}')?;
        }
    }

    f.write_str("'; ")?;
    f.write_fmt(format_args!("{:X?}", data))?;
    f.write_char(')')?;

    Ok(())
}

#[cfg(test)]
mod test {
    use super::*;
    use arrayvec::ArrayString;
    use core::fmt::Write;

    #[test]
    fn test_debug_no_value() {
        let mut buf = ArrayString::<20>::new();

        write!(&mut buf, "{:?}", Response::Ok).expect("Can't write");
        assert_eq!(&buf, "Ok");
    }

    #[test]
    fn test_debug_simple_value() {
        let mut buf = ArrayString::<20>::new();

        write!(&mut buf, "{:?}", Response::Connect(1)).expect("Can't write");
        assert_eq!(&buf, "Connect(1)");
    }

    fn test_debug_data() {
        let mut buf = ArrayString::<256>::new();
        let data = b"FOO\0BAR";

        let mut array = [0u8; super::BUFFER_LEN];
        for (&x, p) in data.iter().zip(array.iter_mut()) {
            *p = x;
        }

        write!(&mut buf, "{:?}", Response::DataReceived(array, data.len())).expect("Can't write");
        assert_eq!(
            &buf,
            "DataReceived(7; 'FOO\\0BAR'; [46, 4F, 4F, 0, 42, 41, 52])"
        );
    }
}
