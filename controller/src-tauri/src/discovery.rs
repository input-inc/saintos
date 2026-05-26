//! mDNS / DNS-SD discovery for SAINT.OS servers.
//!
//! The Steam Deck (and any vanilla Linux without libnss-mdns + a
//! configured systemd-resolved) can't resolve `*.local` hostnames
//! through the OS resolver. Since the controller binary talks to the
//! server over WebSocket, that means a Steam Deck operator can't
//! connect by hostname — only by IP. We can't ship libnss-mdns "in the
//! app," and SteamOS's read-only root makes installing it manually
//! awkward. So we run our own pure-Rust mDNS client INSIDE the
//! controller and bypass the OS resolver entirely.
//!
//! This module provides two capabilities:
//!
//!   1. `resolve_local_hostname(host, timeout)` — one-shot mDNS A
//!      lookup. Used inline by the connect path so an operator who
//!      types `opensaint.local` gets transparent resolution before the
//!      WebSocket library calls getaddrinfo.
//!
//!   2. `DiscoveryService` — a background browser for
//!      `_http._tcp.local.` services. The SAINT.OS server advertises
//!      itself via avahi at install time (see
//!      `.github/dist/install.sh` `setup_mdns`), so the controller
//!      sees every reachable robot and can show the operator a
//!      dropdown instead of asking them to type a hostname.
//!
//! Both paths use multicast UDP on `224.0.0.251:5353` (and the v6
//! equivalent), opened from inside the controller process. No system
//! package, no config file edit, no read-only filesystem dance.

use mdns_sd::{ServiceDaemon, ServiceEvent, ServiceInfo};
use parking_lot::RwLock;
use serde::Serialize;
use std::collections::HashMap;
use std::net::{IpAddr, Ipv4Addr};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

/// The DNS-SD service type the SAINT.OS server advertises through
/// avahi. Must match the `<type>` element written to
/// `/etc/avahi/services/saint-os.service` by `install.sh::setup_mdns`.
const SAINT_SERVICE_TYPE: &str = "_http._tcp.local.";

/// Substring of the service "instance name" that identifies a server
/// as a SAINT.OS instance. avahi expands `replace-wildcards="yes"` on
/// `SAINT.OS on %h` to e.g. `SAINT.OS on opensaint`, so any service
/// instance whose name contains "SAINT.OS" is one of ours. Skipping
/// generic `_http._tcp` services that happen to share the network
/// (printers, NAS boxes) keeps the dropdown free of noise.
const SAINT_NAME_PREFIX: &str = "SAINT.OS";

/// One discovered SAINT.OS instance, in the shape the frontend
/// consumes. Hostname is included alongside ipv4 so the dropdown can
/// show "opensaint (192.168.10.1)" — humans recognize names, the
/// connect path uses the IP.
#[derive(Clone, Debug, Serialize)]
pub struct DiscoveredServer {
    /// The mDNS service-instance name, e.g. "SAINT.OS on opensaint".
    /// This is what the operator-facing label uses if `hostname` is
    /// missing or unreadable.
    pub instance_name: String,
    /// Hostname portion stripped of the trailing `.local.` and the
    /// service-domain dressing — bare "opensaint" rather than the
    /// FQDN. Empty if the service info didn't include a hostname.
    pub hostname: String,
    /// The first IPv4 address advertised in the SRV/A records. IPv6
    /// is intentionally ignored: the SAINT.OS server's HTTP listener
    /// binds to v4 today, so a v6-only path wouldn't connect.
    pub ipv4: Option<Ipv4Addr>,
    /// The port the server advertised (typically 80 from the avahi
    /// service file, but trust whatever is announced).
    pub port: u16,
}

/// Strip the trailing `.` and `.local.` decoration from a mDNS
/// hostname to produce the bare label the UI shows. `opensaint.local.`
/// → `opensaint`. Robust against partial decorations or missing dots.
fn humanize_hostname(raw: &str) -> String {
    let mut h = raw.trim_end_matches('.').to_string();
    if let Some(stripped) = h.strip_suffix(".local") {
        h = stripped.to_string();
    }
    h
}

/// Pull the operator-visible parts of a `ServiceInfo` into our
/// `DiscoveredServer` shape. Returns None if the service is clearly
/// not a SAINT.OS instance (so the caller can skip it). Anything that
/// matches the prefix is included, even if some fields are missing —
/// the dropdown can still surface a partially-populated entry rather
/// than silently dropping a server that's mid-resolution.
fn discovered_from(info: &ServiceInfo) -> Option<DiscoveredServer> {
    let instance_name = info.get_fullname().to_string();
    if !instance_name.contains(SAINT_NAME_PREFIX) {
        return None;
    }
    let hostname = humanize_hostname(info.get_hostname());
    let ipv4 = info
        .get_addresses_v4()
        .iter()
        .copied()
        .map(|a| *a)
        .next();
    Some(DiscoveredServer {
        instance_name,
        hostname,
        ipv4,
        port: info.get_port(),
    })
}

/// Background mDNS browser. Owns the daemon, the worker thread, and a
/// RwLock-protected map keyed by the full service-instance name so
/// re-resolutions update the same entry instead of duplicating it.
pub struct DiscoveryService {
    daemon: ServiceDaemon,
    found: Arc<RwLock<HashMap<String, DiscoveredServer>>>,
}

impl DiscoveryService {
    /// Start a background browse on `_http._tcp.local.` Returns Err
    /// only if the daemon can't open its UDP socket (typically a
    /// permission or address-already-in-use issue) — the browse loop
    /// itself runs forever on its own thread.
    pub fn start() -> Result<Self, String> {
        let daemon =
            ServiceDaemon::new().map_err(|e| format!("mDNS daemon init failed: {}", e))?;
        let receiver = daemon
            .browse(SAINT_SERVICE_TYPE)
            .map_err(|e| format!("mDNS browse start failed: {}", e))?;

        let found = Arc::new(RwLock::new(HashMap::<String, DiscoveredServer>::new()));
        let found_for_thread = found.clone();

        thread::spawn(move || {
            log::info!("mDNS browse started on {}", SAINT_SERVICE_TYPE);
            // recv() blocks until the daemon shuts down, which we
            // never do — the browse runs for the lifetime of the
            // process. Any error here means the daemon died; the
            // worker exits and discoveries become stale, but the rest
            // of the app keeps running on whatever was last seen.
            while let Ok(event) = receiver.recv() {
                match event {
                    ServiceEvent::ServiceResolved(info) => {
                        if let Some(server) = discovered_from(&info) {
                            log::debug!(
                                "mDNS resolved: {} @ {:?}:{}",
                                server.instance_name,
                                server.ipv4,
                                server.port
                            );
                            found_for_thread
                                .write()
                                .insert(server.instance_name.clone(), server);
                        }
                    }
                    ServiceEvent::ServiceRemoved(_ty, fullname) => {
                        log::debug!("mDNS removed: {}", fullname);
                        found_for_thread.write().remove(&fullname);
                    }
                    _ => { /* Found-but-not-resolved-yet, search-stopped: ignored */ }
                }
            }
            log::info!("mDNS browse loop ended");
        });

        Ok(Self { daemon, found })
    }

    /// Snapshot of currently-known SAINT.OS servers, sorted by hostname
    /// for stable UI ordering. Cheap — just clones the entries out of
    /// the lock. The frontend polls this from a Tauri command.
    pub fn snapshot(&self) -> Vec<DiscoveredServer> {
        let mut v: Vec<DiscoveredServer> = self.found.read().values().cloned().collect();
        v.sort_by(|a, b| a.hostname.cmp(&b.hostname));
        v
    }

    /// One-shot resolution of a `.local` hostname to an IPv4 address.
    /// Used by the connect path BEFORE handing the host string to
    /// `tokio-tungstenite`, since that library defers to getaddrinfo
    /// which fails for .local on Steam Deck. Returns None on timeout
    /// or if no v4 address is in the answer.
    ///
    /// Strategy: ask the daemon to resolve the bare hostname directly.
    /// mdns-sd's hostname-resolution path sends an A-record query on
    /// the multicast group and collects answers; we wait up to
    /// `timeout` for the first IPv4 reply.
    pub fn resolve(&self, host: &str, timeout: Duration) -> Option<IpAddr> {
        // Normalize to the FQDN form mdns-sd expects ("opensaint.local.").
        // The library accepts both bare and dotted forms but is more
        // reliable with the trailing dot.
        let normalized = if host.ends_with('.') {
            host.to_string()
        } else if host.ends_with(".local") {
            format!("{}.", host)
        } else {
            format!("{}.local.", host)
        };

        let receiver = match self.daemon.resolve_hostname(&normalized, Some(timeout.as_millis() as u64)) {
            Ok(r) => r,
            Err(e) => {
                log::warn!("mDNS resolve_hostname({}) failed to start: {}", normalized, e);
                return None;
            }
        };

        let deadline = Instant::now() + timeout;
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                log::warn!("mDNS resolve_hostname({}) timed out", normalized);
                return None;
            }
            match receiver.recv_timeout(remaining) {
                Ok(mdns_sd::HostnameResolutionEvent::AddressesFound(_, addrs)) => {
                    if let Some(v4) = addrs.iter().find_map(|a| match a {
                        IpAddr::V4(v) => Some(*v),
                        _ => None,
                    }) {
                        log::info!("mDNS resolved {} → {}", normalized, v4);
                        return Some(IpAddr::V4(v4));
                    }
                    // No v4 in this event — keep waiting for more.
                }
                Ok(_) => { /* SearchStarted / SearchStopped: keep waiting */ }
                Err(_) => {
                    // Timeout or daemon shutdown — bail.
                    return None;
                }
            }
        }
    }
}
