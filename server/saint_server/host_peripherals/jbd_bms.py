"""
JBD-protocol BMS driver — server-side BLE variant.

Mirrors firmware/raspberrypi/saint_node/peripherals/pathfinder_bms.py
and ble_transport.py, but lives inside the saint-server process so
the `host_controller` virtual node can own BLE BMSes directly without
needing a co-located Pi-node service.

Protocol UUIDs (JBD / Xiaoxiang / Overkill stack):

    service       0000ff00-0000-1000-8000-00805f9b34fb
    write (TX)    0000ff02-0000-1000-8000-00805f9b34fb   host → BMS
    notify (RX)   0000ff01-0000-1000-8000-00805f9b34fb   BMS  → host

Wire frame (same in UART and BLE):

    [0xDD] [Action] [Register] [Length] [Data...] [CRC_hi] [CRC_lo] [0x77]

We read register 0x03 (basic info) on a configurable interval. The
parser is identical to the Pi-node copy — both must stay in sync if
the JBD protocol decoder changes. Channels exposed on each instance:

    pack_voltage  (V)         soc           (%)
    current       (A)         remain_cap    (Ah)
    temp_1        (°C)        temp_2        (°C)
    cycles        (count)     protection    (uint16 bitset)

Reconnect: BLE drops are routine (BMS power-cycle, RF interference,
out-of-range). The driver runs an indefinite reconnect loop with
1s/5s/30s/60s backoff so unattended operation survives the BMS going
to sleep or the operator power-cycling a battery pack.
"""

from __future__ import annotations

import asyncio
from typing import Callable, Dict, List, Optional

try:
    from bleak import BleakClient
    BLEAK_AVAILABLE = True
except ImportError:
    BLEAK_AVAILABLE = False


JBD_BLE_SERVICE_UUID  = "0000ff00-0000-1000-8000-00805f9b34fb"
JBD_BLE_WRITE_UUID    = "0000ff02-0000-1000-8000-00805f9b34fb"
JBD_BLE_NOTIFY_UUID   = "0000ff01-0000-1000-8000-00805f9b34fb"

JBD_FRAME_START       = 0xDD
JBD_FRAME_END         = 0x77

# Wire format note: request and response frames have DIFFERENT
# layouts at bytes [1..2].
#
#   REQUEST   [0xDD] [action] [register] [length] [...] [crc_hi] [crc_lo] [0x77]
#   RESPONSE  [0xDD] [register] [status] [length] [...] [crc_hi] [crc_lo] [0x77]
#
# `action` only exists in the request (0xA5 = read, 0x5A = write).
# `register` echoes the request register in the response.
# `status` is 0x00 on success, 0x80 on command error. Easy to swap
# the two positions by accident — earlier versions did exactly that
# and rejected every real reply.
JBD_ACTION_READ       = 0xA5    # request byte[1]
JBD_ACTION_WRITE      = 0x5A    # request byte[1]
JBD_STATUS_OK         = 0x00    # response byte[2]
JBD_STATUS_ERROR      = 0x80    # response byte[2]
JBD_REG_BASIC_INFO    = 0x03  # pack metrics + protection bits
JBD_REG_CELL_VOLTAGES = 0x04  # per-cell mV array (2 bytes per cell)
JBD_RESPONSE_MAX_DATA = 64

# JBD-clone BMSes report up to 16 cells in register 0x04. The catalog
# entry declares cell_01..cell_16 statically; instances with fewer
# series cells (4S, 8S, etc.) just leave the extras at 0.
JBD_MAX_CELLS         = 16

# Basic-info protection bitmask (data bytes 16..17 of register 0x03).
# Bits 13..15 are reserved across the JBD/Overkill/Xiaoxiang lineage;
# we still surface them generically as "fault bit N" if seen so a
# vendor-specific bit never goes silent.
JBD_PROTECTION_BITS: Dict[int, str] = {
    0:  "cell overvoltage",
    1:  "cell undervoltage",
    2:  "pack overvoltage",
    3:  "pack undervoltage",
    4:  "charge overtemperature",
    5:  "charge undertemperature",
    6:  "discharge overtemperature",
    7:  "discharge undertemperature",
    8:  "charge overcurrent",
    9:  "discharge overcurrent",
    10: "short circuit",
    11: "front-end IC error",
    12: "MOS software lock",
}


def decode_protection_bits(bits: int) -> List[str]:
    """Return the list of fault names currently asserted in the JBD
    protection bitmask. Unknown bits are surfaced as ``fault bit N``
    so the operator still sees them in logs."""
    out: List[str] = []
    for bit in range(16):
        if bits & (1 << bit):
            out.append(JBD_PROTECTION_BITS.get(bit, f"fault bit {bit}"))
    return out


def decode_fet_status(byte_val: int) -> str:
    """JBD FET-status byte: bit 0 = charge MOS on, bit 1 = discharge
    MOS on. Returns a short operator-readable summary."""
    charge_on = bool(byte_val & 0x01)
    discharge_on = bool(byte_val & 0x02)
    if charge_on and discharge_on:
        return "charge+discharge ON"
    if charge_on:
        return "charge ON, discharge OFF"
    if discharge_on:
        return "discharge ON, charge OFF"
    return "both OFF"

# Reconnect backoff. Last value caps subsequent retries.
_RECONNECT_BACKOFF_S = (1.0, 5.0, 30.0, 60.0)

# write_then_read timeout: how long we'll wait for a reply frame to
# accumulate via notifications before giving up on a single poll.
_REPLY_TIMEOUT_S      = 2.0

# Cap the notify buffer so a chatty BMS can't grow it unbounded.
_NOTIFY_BUF_CAP       = 1024


# ── Protocol helpers ──────────────────────────────────────────────


def jbd_checksum(data: bytes) -> int:
    """JBD checksum: 0 minus each byte, kept in a 16-bit window."""
    s = 0
    for b in data:
        s = (s - b) & 0xFFFF
    return s


def build_read_request(reg: int) -> bytes:
    """7-byte read request frame."""
    payload = bytes([reg, 0x00])
    c = jbd_checksum(payload)
    return (bytes([JBD_FRAME_START, JBD_ACTION_READ])
            + payload
            + bytes([(c >> 8) & 0xFF, c & 0xFF, JBD_FRAME_END]))


def parse_basic_info_response(frame: bytes) -> Optional[Dict[str, float]]:
    """Validate a JBD basic-info response and return decoded fields.
    Returns None on framing / checksum / length errors.

    Wire format reminder:
        [0xDD] [register] [status] [length] [data...] [crc_hi] [crc_lo] [0x77]

    Register and status occupy positions 1 and 2 respectively — easy
    to mix up with the request layout (which puts the action byte at
    [1] and the register at [2]).
    """
    if len(frame) < 4 + 3:
        return None
    if frame[0] != JBD_FRAME_START or frame[-1] != JBD_FRAME_END:
        return None
    reg = frame[1]
    if reg != JBD_REG_BASIC_INFO:
        return None
    if frame[2] != JBD_STATUS_OK:
        return None
    data_len = frame[3]
    if data_len < 23 or 4 + data_len + 3 > len(frame):
        return None
    data = frame[4: 4 + data_len]
    # Response checksum input is [length, data] — register and status
    # are excluded. Earlier versions also included the register byte
    # (mirroring the request-side checksum); that produces a value
    # exactly `register` higher than what the wire sends, and every
    # real reply got rejected.
    expected = jbd_checksum(bytes([data_len]) + data)
    received = (frame[4 + data_len] << 8) | frame[4 + data_len + 1]
    if expected != received:
        return None

    def i16(off):
        v = (data[off] << 8) | data[off + 1]
        if v & 0x8000:
            v -= 0x10000
        return v

    def u16(off):
        return (data[off] << 8) | data[off + 1]

    pack_v = u16(0) * 0.01            # bytes 0-1, 10mV units
    current = i16(2) * 0.01           # bytes 2-3, signed 10mA
    remain  = u16(4) * 0.01           # bytes 4-5, 10mAh
    full    = u16(6) * 0.01           # bytes 6-7
    cycles  = u16(8)                  # bytes 8-9
    protection = u16(16)              # bytes 16-17 (fault bitmask)
    soc     = data[19]                # byte 19, 0-100
    fet     = data[20] if len(data) > 20 else 0     # bit0=chg, bit1=dchg
    ntc_count = data[22] if len(data) > 22 else 0

    temps = []
    base = 23
    for i in range(min(ntc_count, 4)):
        off = base + i * 2
        if off + 1 >= len(data):
            break
        raw = u16(off)
        celsius = (raw - 2731) / 10.0
        temps.append(celsius)

    return {
        "pack_voltage": pack_v,
        "current": current,
        "remain_cap": remain,
        "full_cap": full,
        "cycles": float(cycles),
        "soc": float(soc),
        "protection": float(protection),
        "fet_status": float(fet),
        "temps": temps,
    }


def parse_cell_voltages_response(frame: bytes) -> Optional[List[float]]:
    """Decode a JBD register-0x04 (cell voltages) reply.

    Same wire envelope as basic info:
        [0xDD] [register=0x04] [status] [length] [cellN_hi cellN_lo ...]
        [crc_hi] [crc_lo] [0x77]

    Each cell is a big-endian uint16 in mV. Returns a list of cell
    voltages in volts, or None on framing / checksum errors."""
    if len(frame) < 4 + 3:
        return None
    if frame[0] != JBD_FRAME_START or frame[-1] != JBD_FRAME_END:
        return None
    if frame[1] != JBD_REG_CELL_VOLTAGES:
        return None
    if frame[2] != JBD_STATUS_OK:
        return None
    data_len = frame[3]
    if data_len < 2 or data_len % 2 != 0:
        return None
    if 4 + data_len + 3 > len(frame):
        return None
    data = frame[4: 4 + data_len]
    expected = jbd_checksum(bytes([data_len]) + data)
    received = (frame[4 + data_len] << 8) | frame[4 + data_len + 1]
    if expected != received:
        return None
    cells: List[float] = []
    for i in range(0, data_len, 2):
        mv = (data[i] << 8) | data[i + 1]
        cells.append(mv / 1000.0)
    return cells


def _has_complete_frame(buf: bytes) -> bool:
    """JBD reply starts 0xDD and ends 0x77. The end byte can legitimately
    appear inside the data section, so we let parse_basic_info_response
    do the real validation — this is just the "stop polling notify"
    signal."""
    if len(buf) < 4:
        return False
    return buf[0] == JBD_FRAME_START and JBD_FRAME_END in buf[2:]


# ── Driver ─────────────────────────────────────────────────────────


# Type alias for the per-channel state-update callback. The server
# wires this to NodeRuntimeState.set_channel under the hood.
StateCallback = Callable[[str, str, float], None]
LogCallback = Optional[Callable[[str, str], None]]


class JbdBleDriver:
    """One BLE-connected JBD BMS. Runs entirely on the caller-provided
    asyncio loop. Pushes decoded telemetry through `state_cb` on every
    successful poll.

    `peripheral_id` is the operator-supplied id (e.g. "pathfinder_bms-1");
    it gets passed through to `state_cb` as the first argument so the
    server can multiplex multiple BMSes into the same NodeRuntimeState
    without per-driver wiring.
    """

    def __init__(self,
                 peripheral_id: str,
                 mac: str,
                 poll_interval_s: float,
                 state_cb: StateCallback,
                 log_cb: LogCallback = None,
                 connected_cb: Optional[Callable[[bool], None]] = None):
        self.peripheral_id = peripheral_id
        self._mac = mac.upper().strip()
        self._poll_interval_s = max(0.1, poll_interval_s)
        self._state_cb = state_cb
        self._log_cb = log_cb
        self._connected_cb = connected_cb

        self._client: Optional["BleakClient"] = None
        self._notify_buf = bytearray()
        self._notify_lock = asyncio.Lock()
        self._reply_event = asyncio.Event()
        self._connected = False
        self._stopping = False
        self._attempt = 0
        self._task: Optional[asyncio.Task] = None

        # Edge-detection state for fault surfacing. `None` until the
        # first successful poll — the very first reading logs the
        # currently-asserted faults (if any) and the FET state so
        # the operator's startup picture is complete; subsequent
        # polls only log when bits change.
        self._last_protection: Optional[int] = None
        self._last_fet_status: Optional[int] = None

    @property
    def mac(self) -> str:
        return self._mac

    @property
    def is_connected(self) -> bool:
        return self._connected

    def update_poll_interval(self, poll_interval_s: float) -> None:
        """Hot-update the poll cadence without restarting the driver."""
        self._poll_interval_s = max(0.1, poll_interval_s)

    async def start(self) -> None:
        """Launch the connect-poll loop. Idempotent — calling twice
        leaves the existing task in place."""
        if self._task and not self._task.done():
            return
        if not BLEAK_AVAILABLE:
            self._log("error",
                "bleak not installed — BLE unavailable. `pip install bleak`")
            return
        self._stopping = False
        self._task = asyncio.create_task(
            self._run(), name=f"jbd-{self.peripheral_id}")

    async def stop(self) -> None:
        """Tear down the connection and cancel the poll task."""
        self._stopping = True
        if self._task and not self._task.done():
            self._task.cancel()
            try:
                await self._task
            except (asyncio.CancelledError, Exception):
                pass
        await self._disconnect()

    # ── internals (run on the host_peripherals asyncio loop) ──────

    async def _run(self) -> None:
        while not self._stopping:
            await self._connect_with_retry()
            if self._stopping:
                break
            try:
                await self._poll_loop()
            except asyncio.CancelledError:
                raise
            except Exception as e:
                self._log("warn",
                    f"poll loop for {self._mac} crashed: {e}; reconnecting")
            self._set_connected(False)
            await self._disconnect()
            # Brief gap before re-entering connect retry — keeps a
            # disconnect-storm from hammering BlueZ.
            await asyncio.sleep(1.0)

    async def _connect_with_retry(self) -> None:
        # Hard ceiling on a single connect attempt. The bleak `timeout`
        # kwarg is best-effort on BlueZ — if the target isn't currently
        # advertising, BleakClient.connect() has been seen to hang well
        # past `timeout` waiting for the device to wake up. wait_for
        # enforces the cap so the retry loop keeps making progress even
        # if the BMS is asleep / out of range.
        connect_deadline_s = 15.0
        while not self._stopping:
            self._log("info",
                f"connecting to {self._mac} "
                f"(attempt {self._attempt + 1})")
            err: Optional[str] = None
            try:
                client = BleakClient(
                    self._mac,
                    timeout=connect_deadline_s,
                    disconnected_callback=self._on_disconnect,
                )
                await asyncio.wait_for(
                    client.connect(), timeout=connect_deadline_s)
                await client.start_notify(
                    JBD_BLE_NOTIFY_UUID, self._on_notify)
                self._client = client
                self._set_connected(True)
                self._attempt = 0
                self._log("info", f"connected to BMS {self._mac}")
                return
            except asyncio.CancelledError:
                raise
            except asyncio.TimeoutError:
                err = f"connect timed out after {connect_deadline_s:.0f}s"
            except Exception as e:
                err = f"{type(e).__name__}: {e}"
            self._client = None
            self._set_connected(False)
            delay = _RECONNECT_BACKOFF_S[
                min(self._attempt, len(_RECONNECT_BACKOFF_S) - 1)]
            self._attempt += 1
            self._log("warn",
                f"connect to {self._mac} failed ({err}); "
                f"retrying in {delay:.0f}s")
            try:
                await asyncio.sleep(delay)
            except asyncio.CancelledError:
                return

    async def _poll_loop(self) -> None:
        while not self._stopping and self._connected:
            # Basic-info: pack metrics + protection state.
            try:
                basic_reply = await self._write_then_read(
                    build_read_request(JBD_REG_BASIC_INFO),
                    _REPLY_TIMEOUT_S)
            except Exception as e:
                self._log("warn",
                    f"BMS {self._mac} basic-info read failed: {e}; "
                    f"dropping link")
                return
            basic = parse_basic_info_response(basic_reply)
            if basic is not None:
                self._push_state(basic)
            elif basic_reply:
                self._log("debug",
                    f"BMS {self._mac} basic-info: {len(basic_reply)} "
                    f"bytes didn't parse")

            # Cell voltages. Failures here are non-fatal — pack-level
            # data is more important than per-cell, so we keep the
            # link up and just skip the cell update this round.
            try:
                cell_reply = await self._write_then_read(
                    build_read_request(JBD_REG_CELL_VOLTAGES),
                    _REPLY_TIMEOUT_S)
            except Exception as e:
                self._log("warn",
                    f"BMS {self._mac} cell-voltage read failed: {e}")
                cell_reply = b""
            cells = parse_cell_voltages_response(cell_reply)
            if cells is not None:
                self._push_cell_state(cells)
            elif cell_reply:
                self._log("debug",
                    f"BMS {self._mac} cell-voltage: {len(cell_reply)} "
                    f"bytes didn't parse")

            await asyncio.sleep(self._poll_interval_s)

    async def _write_then_read(self, tx: bytes, timeout_s: float) -> bytes:
        if self._client is None or not self._connected:
            return b""
        async with self._notify_lock:
            self._notify_buf.clear()
            self._reply_event.clear()
        await self._client.write_gatt_char(
            JBD_BLE_WRITE_UUID, tx, response=False)
        try:
            await asyncio.wait_for(self._reply_event.wait(), timeout_s)
        except asyncio.TimeoutError:
            pass
        async with self._notify_lock:
            return bytes(self._notify_buf)

    def _on_notify(self, _char, data: bytearray) -> None:
        # bleak invokes this on the loop thread. The notify_lock
        # contention here is brief — append + maybe signal.
        self._notify_buf.extend(data)
        if len(self._notify_buf) > _NOTIFY_BUF_CAP:
            del self._notify_buf[: _NOTIFY_BUF_CAP // 2]
        if _has_complete_frame(self._notify_buf):
            self._reply_event.set()

    def _on_disconnect(self, _client) -> None:
        # Bleak fires this on the loop. Flip the state flag so the
        # poll loop's `while self._connected` exits and the outer
        # _run loop re-enters _connect_with_retry.
        if not self._stopping:
            self._log("warn", f"BMS {self._mac} disconnected")
        self._set_connected(False)

    async def _disconnect(self) -> None:
        client, self._client = self._client, None
        self._set_connected(False)
        if client is None:
            return
        try:
            await client.stop_notify(JBD_BLE_NOTIFY_UUID)
        except Exception:
            pass
        try:
            await client.disconnect()
        except Exception:
            pass

    def _set_connected(self, value: bool) -> None:
        if self._connected == value:
            return
        self._connected = value
        if not value:
            # Forget previous fault state so the first poll after
            # the next reconnect re-emits the cumulative picture
            # (BMS may have cleared or asserted faults while we
            # were dropped).
            self._last_protection = None
            self._last_fet_status = None
        if self._connected_cb is not None:
            try:
                self._connected_cb(value)
            except Exception as e:
                self._log("warn", f"connected_cb raised: {e}")

    def _push_state(self, decoded: Dict[str, float]) -> None:
        # Channel names match the BMS catalog entry in
        # peripheral_model.py — keep these two lists aligned.
        self._state_cb(self.peripheral_id, "pack_voltage", decoded["pack_voltage"])
        self._state_cb(self.peripheral_id, "current",      decoded["current"])
        self._state_cb(self.peripheral_id, "soc",          decoded["soc"])
        self._state_cb(self.peripheral_id, "remain_cap",   decoded["remain_cap"])
        self._state_cb(self.peripheral_id, "cycles",       decoded["cycles"])
        self._state_cb(self.peripheral_id, "protection",   decoded["protection"])
        self._state_cb(self.peripheral_id, "fet_status",   decoded.get("fet_status", 0.0))
        temps = decoded["temps"]
        if len(temps) >= 1:
            self._state_cb(self.peripheral_id, "temp_1", temps[0])
        if len(temps) >= 2:
            self._state_cb(self.peripheral_id, "temp_2", temps[1])

        self._surface_alerts(int(decoded["protection"]),
                             int(decoded.get("fet_status", 0)))

    def _push_cell_state(self, cells: List[float]) -> None:
        """Surface each cell voltage as a `cell_NN` channel. Cells
        beyond what this BMS reports are zeroed so a 4S pack doesn't
        leak the previous-instance's stale cell_05..cell_16."""
        for i in range(JBD_MAX_CELLS):
            channel = f"cell_{i + 1:02d}"
            value = cells[i] if i < len(cells) else 0.0
            self._state_cb(self.peripheral_id, channel, value)

    def _surface_alerts(self, prot: int, fet: int) -> None:
        """Emit a log line on each rising / falling edge of the BMS
        protection bitmask, plus on any change to the charge/discharge
        FET state. Cumulative steady-state is logged ONCE at the first
        post-connect poll so a restart shows the operator the current
        picture without nagging on subsequent unchanged polls."""
        if self._last_protection is None:
            if prot != 0:
                names = ", ".join(decode_protection_bits(prot))
                self._log("warn",
                    f"BMS protection asserted on connect: "
                    f"{names} (0x{prot:04x})")
            else:
                self._log("info", "BMS reports no active faults")
        else:
            newly = prot & ~self._last_protection
            cleared = self._last_protection & ~prot
            if newly:
                names = ", ".join(decode_protection_bits(newly))
                self._log("warn",
                    f"BMS new fault: {names} (0x{newly:04x}, "
                    f"now=0x{prot:04x})")
            if cleared:
                names = ", ".join(decode_protection_bits(cleared))
                self._log("info",
                    f"BMS fault cleared: {names} (was 0x{cleared:04x}, "
                    f"now=0x{prot:04x})")
        self._last_protection = prot

        if self._last_fet_status is None:
            self._log("info", f"BMS FETs: {decode_fet_status(fet)}")
        elif fet != self._last_fet_status:
            self._log("warn",
                f"BMS FET state changed: "
                f"{decode_fet_status(self._last_fet_status)} -> "
                f"{decode_fet_status(fet)}")
        self._last_fet_status = fet

    def _log(self, level: str, msg: str) -> None:
        if self._log_cb is not None:
            try:
                self._log_cb(level, f"jbd[{self.peripheral_id}]: {msg}")
            except Exception:
                pass
