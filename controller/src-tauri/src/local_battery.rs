//! The controller's own battery, for the dashboard: a Steam Deck's, or
//! whatever Linux device the app is running on.
//!
//! The robot's packs reach the dashboard as BMS telemetry from the
//! server; the handheld's charge is local and nothing on the robot side
//! knows it. Linux exposes it under `/sys/class/power_supply/<name>/` as
//! one small text file per attribute (BAT1 on a Steam Deck).
//!
//! Reading it is a handful of file reads, cheap enough to poll every few
//! seconds while the dashboard is open. Off Linux, or with no system
//! battery (a desktop dev machine), it reports nothing and the
//! dashboard simply shows no card for it.
//!
//! Nothing here is Steam Deck specific: any Linux handheld or laptop
//! exposes its battery the same way. The card's title comes from the
//! machine's DMI product name, which is how a Deck is recognised.

use serde::Serialize;
use std::fs;
use std::path::{Path, PathBuf};

const POWER_SUPPLY_DIR: &str = "/sys/class/power_supply";
const DMI_DIR: &str = "/sys/class/dmi/id";

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct LocalBattery {
    /// What to call this device on the card, e.g. "Steam Deck".
    pub device_name: String,
    /// Charge, 0–100 %.
    pub percent: Option<f64>,
    /// The kernel's status word: "Charging", "Discharging", "Full",
    /// "Not charging" or "Unknown".
    pub status: String,
    pub charging: bool,
    pub voltage: Option<f64>,
    /// Amps; negative while discharging. ACPI batteries such as the
    /// Deck's report an unsigned magnitude, so the sign comes from
    /// `status`.
    pub current: Option<f64>,
}

/// Read the system battery, if there is one.
pub fn read() -> Option<LocalBattery> {
    let mut b = read_from(Path::new(POWER_SUPPLY_DIR))?;
    b.device_name = device_name_from(Path::new(DMI_DIR));
    Some(b)
}

/// A friendly name for this machine from its DMI product name. Valve's
/// boards are named by codename (Jupiter = Steam Deck LCD, Galileo =
/// Steam Deck OLED); other vendors usually put the model there. Firmware
/// placeholder strings fall back to a generic name.
pub fn device_name_from(dmi: &Path) -> String {
    const GENERIC: &str = "This controller";
    let product = fs::read_to_string(dmi.join("product_name"))
        .map(|s| s.trim().to_string())
        .unwrap_or_default();
    match product.as_str() {
        "Jupiter" | "Galileo" => "Steam Deck".to_string(),
        "" | "Default string" | "To Be Filled By O.E.M." | "System Product Name"
            | "Not Applicable" => GENERIC.to_string(),
        name => name.to_string(),
    }
}

/// `read` against an arbitrary power_supply directory, for tests. The
/// device name is left generic; `read` fills it in.
pub fn read_from(root: &Path) -> Option<LocalBattery> {
    let dir = find_system_battery(root)?;
    let attr = |name: &str| -> Option<String> {
        fs::read_to_string(dir.join(name)).ok().map(|s| s.trim().to_string())
    };
    let num = |name: &str| -> Option<f64> { attr(name)?.parse::<f64>().ok() };

    let status = attr("status").unwrap_or_else(|| "Unknown".to_string());
    let charging = status == "Charging";
    let voltage = num("voltage_now").map(|uv| uv / 1_000_000.0);

    // current_now is µA. Some ACPI batteries only report power_now (µW),
    // in which case current = power / voltage.
    let magnitude = num("current_now").map(|ua| ua.abs() / 1_000_000.0).or_else(|| {
        let watts = num("power_now")?.abs() / 1_000_000.0;
        let volts = voltage.filter(|v| *v > 0.0)?;
        Some(watts / volts)
    });
    let current = magnitude.map(|a| if status == "Discharging" { -a } else { a });

    let percent = num("capacity").map(|p| p.clamp(0.0, 100.0)).or_else(|| {
        // Fall back to charge or energy counters.
        let (now, full) = num("charge_now")
            .zip(num("charge_full"))
            .or_else(|| num("energy_now").zip(num("energy_full")))?;
        (full > 0.0).then(|| (now / full * 100.0).clamp(0.0, 100.0))
    });

    Some(LocalBattery {
        device_name: "This controller".to_string(),
        percent,
        status,
        charging,
        voltage,
        current,
    })
}

/// The first power supply of type "Battery" that powers the system.
/// Peripherals (a paired gamepad, a mouse) also appear as batteries but
/// carry `scope` = "Device", so they are skipped.
fn find_system_battery(root: &Path) -> Option<PathBuf> {
    let mut entries: Vec<PathBuf> = fs::read_dir(root)
        .ok()?
        .filter_map(|e| e.ok().map(|e| e.path()))
        .collect();
    entries.sort(); // deterministic choice when there are several
    entries.into_iter().find(|dir| {
        let read = |name: &str| {
            fs::read_to_string(dir.join(name)).map(|s| s.trim().to_string()).ok()
        };
        read("type").as_deref() == Some("Battery") && read("scope").as_deref() != Some("Device")
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU32, Ordering};

    /// A throwaway power_supply tree: `supplies` is (name, [(attr, value)]).
    fn fake_root(supplies: &[(&str, &[(&str, &str)])]) -> PathBuf {
        static N: AtomicU32 = AtomicU32::new(0);
        let root = std::env::temp_dir().join(format!(
            "saint-local-battery-{}-{}",
            std::process::id(),
            N.fetch_add(1, Ordering::SeqCst)
        ));
        let _ = fs::remove_dir_all(&root);
        for (name, attrs) in supplies {
            let dir = root.join(name);
            fs::create_dir_all(&dir).unwrap();
            for (k, v) in *attrs {
                fs::write(dir.join(k), format!("{}\n", v)).unwrap();
            }
        }
        root
    }

    #[test]
    fn reads_a_steam_deck_style_battery() {
        let root = fake_root(&[
            ("ACAD", &[("type", "Mains"), ("online", "0")]),
            ("BAT1", &[
                ("type", "Battery"),
                ("status", "Discharging"),
                ("capacity", "73"),
                ("voltage_now", "8150000"),
                ("current_now", "1200000"),
            ]),
        ]);
        let b = read_from(&root).expect("battery");
        assert_eq!(b.percent, Some(73.0));
        assert_eq!(b.status, "Discharging");
        assert!(!b.charging);
        assert_eq!(b.voltage, Some(8.15));
        // Unsigned magnitude in sysfs; discharging makes it negative.
        assert_eq!(b.current, Some(-1.2));
    }

    #[test]
    fn charging_current_is_positive() {
        let root = fake_root(&[("BAT1", &[
            ("type", "Battery"), ("status", "Charging"), ("capacity", "40"),
            ("voltage_now", "8000000"), ("current_now", "2000000"),
        ])]);
        let b = read_from(&root).unwrap();
        assert!(b.charging);
        assert_eq!(b.current, Some(2.0));
    }

    #[test]
    fn derives_current_from_power_when_current_is_missing() {
        let root = fake_root(&[("BAT0", &[
            ("type", "Battery"), ("status", "Discharging"), ("capacity", "50"),
            ("voltage_now", "8000000"), ("power_now", "16000000"),
        ])]);
        assert_eq!(read_from(&root).unwrap().current, Some(-2.0));
    }

    #[test]
    fn derives_percent_from_energy_when_capacity_is_missing() {
        let root = fake_root(&[("BAT0", &[
            ("type", "Battery"), ("status", "Full"),
            ("energy_now", "30000000"), ("energy_full", "40000000"),
        ])]);
        assert_eq!(read_from(&root).unwrap().percent, Some(75.0));
    }

    #[test]
    fn skips_peripheral_batteries() {
        // A paired gamepad shows up as a Battery with scope=Device.
        let root = fake_root(&[("hid-gamepad-battery", &[
            ("type", "Battery"), ("scope", "Device"), ("capacity", "90"),
        ])]);
        assert_eq!(read_from(&root), None);
    }

    #[test]
    fn names_the_device_from_dmi() {
        let name = |product: &str| {
            let root = fake_root(&[("id", &[("product_name", product)])]);
            device_name_from(&root.join("id"))
        };
        assert_eq!(name("Jupiter"), "Steam Deck");
        assert_eq!(name("Galileo"), "Steam Deck");
        assert_eq!(name("ROG Ally RC71L_RC71L"), "ROG Ally RC71L_RC71L");
        assert_eq!(name("To Be Filled By O.E.M."), "This controller");
        assert_eq!(device_name_from(Path::new("/nonexistent/dmi")), "This controller");
    }

    #[test]
    fn no_battery_reports_nothing() {
        let root = fake_root(&[("ACAD", &[("type", "Mains")])]);
        assert_eq!(read_from(&root), None);
        assert_eq!(read_from(Path::new("/nonexistent/power_supply")), None);
    }
}
