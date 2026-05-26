// Propagate controller/VERSION into the three downstream manifests that
// each consume their own copy of the version string:
//
//   - package.json            (npm + Angular)
//   - src-tauri/Cargo.toml    (cargo, for the Tauri binary)
//   - src-tauri/tauri.conf.json (Tauri bundler / runtime API)
//
// controller/VERSION is the single source of truth. This script is wired
// into `npm run build` and `npm run tauri:build` via package.json so any
// bump to VERSION lands in every artifact without a manual step.

const fs = require('fs');
const path = require('path');

const ROOT = path.resolve(__dirname, '..');
const VERSION_FILE = path.join(ROOT, 'VERSION');

const version = fs.readFileSync(VERSION_FILE, 'utf8').trim();
if (!/^\d+\.\d+\.\d+(?:[.+-][A-Za-z0-9.+-]+)?$/.test(version)) {
    console.error(`[sync-version] invalid version in ${VERSION_FILE}: "${version}"`);
    process.exit(1);
}

function updateJson(file, key) {
    const data = JSON.parse(fs.readFileSync(file, 'utf8'));
    if (data[key] === version) return false;
    data[key] = version;
    fs.writeFileSync(file, JSON.stringify(data, null, 2) + '\n');
    return true;
}

function updateCargoToml(file) {
    const text = fs.readFileSync(file, 'utf8');
    // Replace only the [package] section's `version = "..."` line. Lazy
    // match between `[package]` and the next `version =` ensures dependency
    // entries further down the file are not touched.
    const updated = text.replace(
        /^(\[package\][\s\S]*?\nversion\s*=\s*)"[^"]+"/m,
        `$1"${version}"`
    );
    if (updated === text) return false;
    fs.writeFileSync(file, updated);
    return true;
}

const results = [
    ['package.json',              updateJson(path.join(ROOT, 'package.json'), 'version')],
    ['src-tauri/tauri.conf.json', updateJson(path.join(ROOT, 'src-tauri/tauri.conf.json'), 'version')],
    ['src-tauri/Cargo.toml',      updateCargoToml(path.join(ROOT, 'src-tauri/Cargo.toml'))],
];

const changed = results.filter(([, c]) => c).map(([f]) => f);
if (changed.length) {
    console.log(`[sync-version] ${version} → ${changed.join(', ')}`);
} else {
    console.log(`[sync-version] ${version} (already in sync)`);
}
