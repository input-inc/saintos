# SAINT.OS Plymouth Boot Splash

Replaces the stock Pi-logo boot splash with a SAINT.OS-branded one.

## Files

| File | Purpose |
|---|---|
| `saint-os.plymouth` | Theme manifest — Plymouth's entry point, points at the script |
| `saint-os.script`   | Renders the splash (centered logo on black background) |
| `opensaint.png`     | Logo, **bundled into the firmware zip by `package.sh`** from the canonical source at `firmware/raspberrypi/assets/opensaint.png`. You don't edit it here. |

## Adding / updating the splash image

The single source of truth is `firmware/raspberrypi/assets/opensaint.png`.
`package.sh` copies it into this directory at zip-build time, and
`scripts/build-local-dist.sh` separately syncs it into
`server/web/public/opensaint.png` so the SPA splash uses the same
logo. To change the splash:

```bash
cp my-new-logo.png firmware/raspberrypi/assets/opensaint.png
scripts/build-local-dist.sh
```

Sizing guidance:

- Pre-size for the **target screen** (Plymouth doesn't scale). Common
  kiosk targets:
  - 1080p landscape: design for 1920×1080
  - Pi-7" official touch: 800×480
  - Waveshare 5.5" portrait kiosk (the Console node): 1440×2560
- **Transparent background** preferred — the script paints solid black
  behind it, so transparency reads as the centered logo over black.
- Anti-alias edges. The Pi's KMS framebuffer doesn't do sub-pixel
  smoothing in the boot phase, so soft edges look better than hard.

If `splash.png` is missing, `package.sh` will warn but still produce
the firmware zip. `install.sh` skips installing the theme when the
image is absent.

## Where it lands on the Pi

The firmware installer copies this whole directory to
`/usr/share/plymouth/themes/saint-os/` and then runs:

```
plymouth-set-default-theme -R saint-os
```

The `-R` rebuilds the initramfs so the theme also covers the very
early boot phase (before the rootfs is mounted), not just post-systemd.

## Restoring the default Pi theme

```
sudo plymouth-set-default-theme -R pix
```

## See also

- `firmware/raspberrypi/scripts/install.sh` (search for "plymouth") —
  where the install step lives.
- Plymouth scripting reference: https://www.freedesktop.org/wiki/Software/Plymouth/Scripts/
