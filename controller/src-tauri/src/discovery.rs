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
//!
//! ## Lifetime
//!
//! The daemon is created ON DEMAND and shut down again once nothing has
//! used it for `IDLE_SHUTDOWN`. It used to be started at app boot and
//! live for the whole session, which for a battery-powered handheld is
//! the wrong default: an open mDNS socket is joined to the multicast
//! group, so the NIC delivers every mDNS packet on the LAN to this
//! process, and received multicast is what stops a Wi-Fi radio reaching
//! its deeper power-save states. The daemon also ran a 5-second
//! interface-change poll of its own the whole time.
//!
//! None of that is needed once we have an address. `.local` resolution
//! happens once per explicit connect — the resolved IP is baked into the
//! WebSocket URL and the reconnect loop reuses it — and the server
//! picker is only on screen while the operator is disconnected and
//! looking at Settings. So both callers touch the daemon in bursts, and
//! between bursts there is nothing to keep open.
//!
//! When idle this module holds no sockets, no threads and no timers.

use mdns_sd::{ServiceDaemon, ServiceEvent, ServiceInfo};
use parking_lot::{Mutex, RwLock};
use serde::Serialize;
use std::collections::HashMap;
use std::net::{IpAddr, Ipv4Addr};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

/// How long the daemon may sit unused before it is shut down.
///
/// Must comfortably exceed the Settings view's discovery poll interval
/// (3 s) or an open picker would thrash the daemon up and down. Also the
/// window in which a connect that follows a browse reuses the same
/// daemon instead of paying startup twice.
const IDLE_SHUTDOWN: Duration = Duration::from_secs(30);

/// How long a freshly started browse is given to collect answers before
/// the first `snapshot()` is expected to show anything. Purely
/// informational — callers poll — but it documents why the first poll
/// after activation comes back empty.
const BROWSE_WARMUP_HINT: Duration = Duration::from_secs(3);

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
/// Whether a service-instance name belongs to a SAINT.OS server.
///
/// The browse type is the generic `_http._tcp.local.`, so the daemon
/// hands us every HTTP service on the LAN — printers, routers, NAS
/// boxes, casting targets. Both browse arms have to apply this, or
/// foreign devices leak into whichever one forgets.
fn is_saint_instance(instance_name: &str) -> bool {
    instance_name.contains(SAINT_NAME_PREFIX)
}

fn discovered_from(info: &ServiceInfo) -> Option<DiscoveredServer> {
    let instance_name = info.get_fullname().to_string();
    if !is_saint_instance(&instance_name) {
        return None;
    }
    let hostname = humanize_hostname(info.get_hostname());
    let ipv4 = info
        .get_addresses_v4()
        .iter()
        .copied()
        .copied()
        .next();
    Some(DiscoveredServer {
        instance_name,
        hostname,
        ipv4,
        port: info.get_port(),
    })
}

/// On-demand mDNS browser + resolver.
///
/// Owns a daemon only while something is using it; see the module docs
/// for why. `found` and `last_used` outlive individual daemons so a
/// browse that restarts doesn't lose the shape of its API.
pub struct DiscoveryService {
    /// The live daemon, if any. `None` means fully idle: no socket, no
    /// threads, no timers.
    daemon: Arc<Mutex<Option<ServiceDaemon>>>,
    found: Arc<RwLock<HashMap<String, DiscoveredServer>>>,
    /// Last time any caller needed discovery. Drives idle shutdown.
    last_used: Arc<Mutex<Instant>>,
}

impl DiscoveryService {
    /// Create the service WITHOUT starting anything. Infallible now:
    /// there is no socket to fail to open until something actually
    /// discovers, and a failure then is reported by that call.
    ///
    /// (This used to open the socket eagerly and return Err if the host
    /// wouldn't allow it — notably the Steam Deck Game Mode sandbox.
    /// That failure is now surfaced per-operation instead, which also
    /// means a host that blocks multicast at boot but not later gets a
    /// working picker instead of a permanently disabled one.)
    pub fn new() -> Self {
        Self {
            daemon: Arc::new(Mutex::new(None)),
            found: Arc::new(RwLock::new(HashMap::new())),
            last_used: Arc::new(Mutex::new(Instant::now())),
        }
    }

    /// Hand back a live daemon, starting one if needed, and mark it used.
    ///
    /// Returns a clone rather than a guard on purpose: `resolve()` blocks
    /// for up to its timeout, and holding the lock across that would
    /// stall the Settings poll behind a connect attempt.
    fn acquire(&self) -> Result<ServiceDaemon, String> {
        *self.last_used.lock() = Instant::now();

        let mut slot = self.daemon.lock();
        if let Some(daemon) = slot.as_ref() {
            return Ok(daemon.clone());
        }

        let daemon =
            ServiceDaemon::new().map_err(|e| format!("mDNS daemon init failed: {}", e))?;

        // Browsing is what populates the picker. Start it with the
        // daemon: a resolve-only caller pays one extra PTR query, and in
        // exchange a connect attempt warms the cache that the picker
        // reads a moment later.
        match daemon.browse(SAINT_SERVICE_TYPE) {
            Ok(receiver) => {
                let found = self.found.clone();
                thread::spawn(move || {
                    log::info!(
                        "mDNS browse started on {} (results in ~{:?})",
                        SAINT_SERVICE_TYPE,
                        BROWSE_WARMUP_HINT
                    );
                    // recv() ends when the daemon shuts down, which is
                    // now a normal event rather than process exit — the
                    // idle reaper does it. The thread just ends with it.
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
                                    found.write().insert(server.instance_name.clone(), server);
                                }
                            }
                            ServiceEvent::ServiceRemoved(_ty, fullname) => {
                                // Filter exactly as the resolve arm does.
                                // This used to log and take the map's
                                // write lock for EVERY `_http._tcp`
                                // service that left the network — a
                                // sleeping printer, a phone walking out
                                // of range — none of which was ever in
                                // our map to remove. Harmless work, but
                                // it wrote other people's devices into
                                // our diagnostic log.
                                if !is_saint_instance(&fullname) {
                                    continue;
                                }
                                log::debug!("mDNS removed: {}", fullname);
                                found.write().remove(&fullname);
                            }
                            _ => { /* Found-but-not-resolved-yet, search-stopped: ignored */ }
                        }
                    }
                    log::info!("mDNS browse loop ended");
                });
            }
            Err(e) => {
                // A daemon without a browse still resolves hostnames,
                // which is the capability the connect path depends on.
                // Degrade to that rather than failing the whole call.
                log::warn!("mDNS browse start failed ({}); resolve-only", e);
            }
        }

        self.spawn_reaper();
        *slot = Some(daemon.clone());
        Ok(daemon)
    }

    /// Shut the daemon down once `IDLE_SHUTDOWN` passes with no use.
    ///
    /// Lives only as long as the daemon does — it exits after shutting
    /// one down, so an idle app has no timer thread at all. That matters:
    /// a permanent reaper ticking away would have cost about what the
    /// daemon's own interface poll did, and defeated the point.
    fn spawn_reaper(&self) {
        let daemon = self.daemon.clone();
        let found = self.found.clone();
        let last_used = self.last_used.clone();

        thread::spawn(move || loop {
            let idle_for = last_used.lock().elapsed();
            if idle_for < IDLE_SHUTDOWN {
                // Sleep only as long as it would take to become idle.
                thread::sleep(IDLE_SHUTDOWN - idle_for);
                continue;
            }

            let mut slot = daemon.lock();
            // Re-check under the lock: acquire() may have handed the
            // daemon out between the idle test and here.
            if last_used.lock().elapsed() < IDLE_SHUTDOWN {
                continue;
            }
            if let Some(d) = slot.take() {
                match d.shutdown() {
                    Ok(_) => log::info!("mDNS daemon idle for {:?}; shut down", idle_for),
                    Err(e) => log::warn!("mDNS daemon shutdown failed: {}", e),
                }
            }
            // Results belong to the daemon that found them; serving them
            // after shutdown would show servers we are no longer tracking
            // and cannot notice leaving.
            found.write().clear();
            return;
        });
    }

    /// Snapshot of currently-known SAINT.OS servers, sorted by hostname
    /// for stable UI ordering.
    ///
    /// Starts the daemon if it isn't running, so the first call after an
    /// idle period returns empty and fills in over ~`BROWSE_WARMUP_HINT`
    /// as answers arrive. The Settings view polls, so that resolves
    /// itself; each poll also counts as "used" and keeps the daemon up
    /// while the picker is on screen.
    pub fn snapshot(&self) -> Vec<DiscoveredServer> {
        if let Err(e) = self.acquire() {
            log::warn!("mDNS discovery unavailable: {}", e);
            return Vec::new();
        }
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
        let daemon = match self.acquire() {
            Ok(d) => d,
            Err(e) => {
                log::warn!("mDNS resolve({}) unavailable: {}", host, e);
                return None;
            }
        };

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

        let receiver = match daemon.resolve_hostname(&normalized, Some(timeout.as_millis() as u64)) {
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

impl Default for DiscoveryService {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The browse type is generic, so this predicate is the only thing
    /// separating our servers from every other HTTP service on the LAN.
    /// Both browse arms depend on it agreeing with itself.
    #[test]
    fn saint_instances_are_recognized() {
        assert!(is_saint_instance("SAINT.OS on opensaint._http._tcp.local."));
        assert!(is_saint_instance("Left Track SAINT.OS._http._tcp.local."));
    }

    /// Construction must not touch the network. The whole point of the
    /// on-demand lifetime is that an app that never opens Settings and
    /// connects by IP never opens a multicast socket at all.
    #[test]
    fn new_starts_nothing() {
        let d = DiscoveryService::new();
        assert!(d.daemon.lock().is_none(), "new() must not start a daemon");
        assert!(d.found.read().is_empty());
    }

    /// The reaper clears results when it shuts a daemon down. Serving a
    /// stale list afterwards would advertise servers we are no longer
    /// tracking and could not notice leaving.
    #[test]
    fn idle_shutdown_window_exceeds_the_settings_poll() {
        // SettingsView polls discover_servers every 3 s while
        // disconnected; each poll counts as use. If the idle window were
        // shorter the picker would thrash the daemon up and down.
        assert!(
            IDLE_SHUTDOWN >= Duration::from_secs(10),
            "idle window must comfortably exceed the 3 s discovery poll",
        );
    }

    #[test]
    fn foreign_http_services_are_rejected() {
        for name in [
            "Brother HL-L2350DW._http._tcp.local.",
            "living-room-tv._http._tcp.local.",
            "synology._http._tcp.local.",
            "saint.os lowercase._http._tcp.local.",
        ] {
            assert!(!is_saint_instance(name), "must reject {name}");
        }
    }
}
