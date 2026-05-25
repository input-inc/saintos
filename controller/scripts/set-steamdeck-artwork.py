#!/usr/bin/env python3
"""Install Steam library art (hero + capsule) for the SAINT Controller
non-Steam game.

Steam stores per-game library art under
``~/.local/share/Steam/userdata/<user-id>/config/grid/``, named after
the non-Steam game's ``appid`` — a number Steam assigns when the
operator adds the controller via *Add a Non-Steam Game*. That appid is
buried in ``shortcuts.vdf`` in the same userdata directory, a binary
key-value file Valve uses for non-Steam shortcuts.

This script:
  1. Walks ``~/.local/share/Steam/userdata/*/config/shortcuts.vdf``
  2. Finds an entry whose ``AppName`` contains the configured pattern
     (default ``SAINT Controller``).
  3. Reads its appid.
  4. Copies the bundled art into ``<userdata>/config/grid/`` with the
     filenames Steam looks for at render time:
       <appid>p.png      → vertical capsule (600 × 900)
       <appid>_hero.png  → hero banner (1920 × 620)
       <appid>.png       → horizontal capsule (920 × 430)

It's idempotent — re-running just overwrites the same files. If the
operator hasn't added the controller as a Non-Steam Game yet, the
script exits with code 1 and a "no entry matching" message; that's
expected, not an error in the build pipeline. ``build.sh`` swallows
the non-zero exit so a first-time install where the operator hasn't
done the Steam add-step yet doesn't fail the whole build.

Usage:
  set-steamdeck-artwork.py [--name-pattern PATTERN]
                           [--art-dir DIR]

The default art-dir resolves to the Flatpak-installed path
(/app/share/com.saintos.Controller/art) when running inside the
sandbox, falling back to ../images/ when running directly from the
repo. Pass --art-dir to override.
"""

import argparse
import shutil
import struct
import sys
from pathlib import Path


# Binary VDF tag bytes (Valve's KeyValues format, binary flavor).
# See https://developer.valvesoftware.com/wiki/KeyValues.
VDF_TYPE_MAP = 0x00
VDF_TYPE_STRING = 0x01
VDF_TYPE_INT = 0x02
VDF_TYPE_END = 0x08


def read_cstring(buf: bytes, offset: int) -> tuple[str, int]:
    """Read a null-terminated UTF-8 string from buf starting at offset.

    Decoded with errors="replace" rather than strict — operator-supplied
    AppNames sometimes contain non-UTF8 bytes from copy-paste, and the
    rest of the file is still parseable around them.
    """
    end = buf.index(0, offset)
    return buf[offset:end].decode("utf-8", errors="replace"), end + 1


def parse_vdf(buf: bytes, offset: int = 0) -> tuple[dict, int]:
    """Parse a binary VDF map starting at offset.

    Returns the parsed map (string keys → dict | str | int values) and
    the offset just past the closing 0x08. Recursively handles nested
    maps. Unknown tag bytes log a warning and stop parsing the current
    map — better to surface partial data than mis-parse junk as valid.
    """
    result: dict = {}
    while offset < len(buf):
        tag = buf[offset]
        offset += 1
        if tag == VDF_TYPE_END:
            return result, offset
        key, offset = read_cstring(buf, offset)
        if tag == VDF_TYPE_MAP:
            value, offset = parse_vdf(buf, offset)
            result[key] = value
        elif tag == VDF_TYPE_STRING:
            value, offset = read_cstring(buf, offset)
            result[key] = value
        elif tag == VDF_TYPE_INT:
            (value,) = struct.unpack_from("<i", buf, offset)
            offset += 4
            result[key] = value
        else:
            print(f"warn: unknown VDF tag 0x{tag:02x} at offset {offset - 1}",
                  file=sys.stderr)
            return result, offset
    return result, offset


def find_shortcuts_dirs() -> list[Path]:
    """All per-user userdata directories that contain shortcuts.vdf.

    Multi-user Decks aren't common but handle them anyway — install
    art for every user that has a matching shortcut.
    """
    base = Path.home() / ".local" / "share" / "Steam" / "userdata"
    if not base.is_dir():
        return []
    result: list[Path] = []
    for child in base.iterdir():
        sv = child / "config" / "shortcuts.vdf"
        if sv.is_file():
            result.append(child)
    return result


def _slugify(s: str) -> str:
    """Collapse a name to lowercase alphanumeric so separator variants
    (space, hyphen, underscore, dot) all compare equal:

        "SAINT Controller"   → "saintcontroller"
        "saint-controller"   → "saintcontroller"     ← what Steam auto-fills
        "com.saintos.Controller" → "comsaintoscontroller"

    Steam's "Add a Non-Steam Game" dialog often names entries from the
    executable filename (which yields a slug-cased name), while
    operators who add it by hand tend to type the human-readable form.
    Matching after slugification accepts both without making the
    operator remember which exact form they used.
    """
    return "".join(c.lower() for c in s if c.isalnum())


def find_entry(shortcuts_vdf: Path, name_pattern: str) -> tuple[int | None, dict | None, list[str]]:
    """Locate the shortcut for the controller.

    Returns (appid, entry_dict, seen_names) where:
      - appid: the Steam-assigned non-Steam-game id, or None if no match
      - entry_dict: the matching shortcut entry, or None
      - seen_names: every AppName we observed in the file (for diagnostics)

    Matching is slugified substring: needle and haystack are both
    reduced to lowercase alphanumeric before the `in` check, so
    "SAINT Controller" finds "saint-controller" and vice versa.
    """
    seen: list[str] = []
    buf = shortcuts_vdf.read_bytes()
    # Steam writes the root key as "Shortcuts" (capital S) in files
    # created via Add-a-Non-Steam-Game. The Valve binary-KV spec is
    # case-insensitive on key names, so accept both casings instead of
    # hard-coding lowercase. Without this we silently fail with a
    # "no shortcut matching" message before even parsing the entries.
    lower = buf[:11].lower()
    if not lower.startswith(b"\x00shortcuts\x00"):
        return None, None, seen
    parsed, _ = parse_vdf(buf, 11)
    needle = _slugify(name_pattern)
    for entry in parsed.values():
        if not isinstance(entry, dict):
            continue
        appname = entry.get("AppName") or entry.get("appname") or ""
        if not isinstance(appname, str):
            continue
        seen.append(appname)
        if needle and needle in _slugify(appname):
            appid = entry.get("appid")
            if isinstance(appid, int):
                return appid, entry, seen
    return None, None, seen


def install_artwork(userdata_dir: Path, appid: int, art_dir: Path) -> list[str]:
    """Copy art into <userdata>/config/grid/ with Steam's expected
    filenames. Steam's grid path uses the UNSIGNED 32-bit form of the
    shortcuts.vdf appid; the VDF stores it signed. Convert before
    formatting filenames.
    """
    appid_u = appid + (1 << 32) if appid < 0 else appid

    grid = userdata_dir / "config" / "grid"
    grid.mkdir(parents=True, exist_ok=True)

    copies = [
        ("libraryCapsule.png", f"{appid_u}p.png"),
        ("libraryHero.png", f"{appid_u}_hero.png"),
        ("horizontalCapsule.png", f"{appid_u}.png"),
    ]
    installed: list[str] = []
    for src_name, dst_name in copies:
        src = art_dir / src_name
        if not src.is_file():
            print(f"warn: source missing: {src}", file=sys.stderr)
            continue
        dst = grid / dst_name
        shutil.copyfile(src, dst)
        installed.append(str(dst))
    return installed


def default_art_dir() -> Path:
    """Look for bundled art first (Flatpak install path); fall back
    to the repo's controller/images/ when running from a dev clone.
    """
    bundled = Path("/app/share/com.saintos.Controller/art")
    if bundled.is_dir():
        return bundled
    here = Path(__file__).resolve()
    repo_art = here.parent.parent / "images"
    if repo_art.is_dir():
        return repo_art
    return bundled   # last resort; will fail loudly at copy time


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--name-pattern", default="SAINT Controller",
                    help='Substring to match against shortcuts.vdf AppName '
                         '(default: "SAINT Controller")')
    ap.add_argument("--art-dir", type=Path, default=None,
                    help="Directory containing libraryCapsule.png, "
                         "libraryHero.png, and horizontalCapsule.png "
                         "(default: bundled Flatpak path, falling back "
                         "to ../images/)")
    args = ap.parse_args()

    art_dir = args.art_dir or default_art_dir()
    userdatas = find_shortcuts_dirs()
    if not userdatas:
        print("No Steam userdata directories found.")
        print(f"  expected: {Path.home()}/.local/share/Steam/userdata/<id>/config/shortcuts.vdf")
        print("  Steam may not be installed, or no non-Steam game has been added yet.")
        return 1

    print(f"Installing artwork from: {art_dir}")
    any_installed = False
    all_seen: list[tuple[str, list[str]]] = []   # (user_dir, [AppNames])
    for ud in userdatas:
        sv = ud / "config" / "shortcuts.vdf"
        appid, entry, seen = find_entry(sv, args.name_pattern)
        all_seen.append((ud.name, seen))
        if not appid:
            print(f"  user {ud.name}: no shortcut matching '{args.name_pattern}'")
            continue
        installed = install_artwork(ud, appid, art_dir)
        if installed:
            any_installed = True
            name = entry.get("AppName", "<unknown>") if entry else "<unknown>"
            print(f"  user {ud.name}: installed art for '{name}' "
                  f"(appid={appid})")
            for p in installed:
                print(f"    → {p}")

    if not any_installed:
        print()
        print("No art installed. The following non-Steam-game AppNames"
              " were found in shortcuts.vdf:")
        any_seen = False
        for ud_name, names in all_seen:
            if names:
                any_seen = True
                print(f"  user {ud_name}:")
                for n in names:
                    print(f"    - {n!r}")
        if not any_seen:
            print("  (none — no Non-Steam Games added yet)")
        print()
        print("To set up:")
        print(f"  1. Open Steam in Desktop Mode")
        print(f"  2. Games → Add a Non-Steam Game → Browse to the controller binary")
        print(f"  3. Re-run this script")
        print(f"  (matching is space/hyphen/underscore-insensitive — 'saint-controller'")
        print(f"   and 'SAINT Controller' both match the default '--name-pattern'.")
        print(f"   Pass --name-pattern <substring> if you used a different name.)")
        return 1

    print()
    print("Restart Steam (or Game Mode) to pick up the new art.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
