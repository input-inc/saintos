# SPA Splash Image — `opensaint.png`

This directory's `opensaint.png` is **auto-synced from
`firmware/raspberrypi/assets/opensaint.png`** by
`scripts/build-local-dist.sh` before each Vite build, so you don't
edit it here — edit the firmware-side copy.

`server/web/index.html` references `/opensaint.png` (Vite serves
`public/` at the URL root in both dev and built modes). The SPA's
`createApp().mount('#app')` replaces the splash element when the
first view paints.

## To update the splash

```bash
# Replace the canonical asset:
cp my-new-logo.png firmware/raspberrypi/assets/opensaint.png

# Rebuild — sync + Vite build happen automatically:
scripts/build-local-dist.sh
```

The same `opensaint.png` also drives the Pi boot splash via the
Plymouth theme at `firmware/raspberrypi/assets/plymouth/saint-os/`.
One file, both surfaces.

## Manual sync (without a full dist build)

```bash
cp firmware/raspberrypi/assets/opensaint.png server/web/public/opensaint.png
cd server/web && npm run build
```

## Sizing

- Square or near-square aspect ratio — CSS clamps to
  `max-width: 60vw; max-height: 50vh; object-fit: contain;` so wide
  images get letterboxed.
- Transparent background recommended — the splash paints solid black
  behind the image.
- For the Plymouth side, Plymouth doesn't scale, so pre-size for the
  framebuffer if the kiosk is portrait (e.g., 1440×2560). The SPA's
  CSS adapts to any viewport, so a square crop (1024×1024) often
  works well for both.
