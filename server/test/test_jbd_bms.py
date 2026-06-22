"""Unit tests for the JBD-protocol BMS frame parsers.

These are pure decoders of battery telemetry off the wire — a parsing
bug here surfaces wrong pack voltage / SOC / fault state to the operator
(a safety signal), so the framing/checksum/field-decode paths are worth
locking down. Mirrors the firmware-side parser tests (FAS100 / RoboClaw).

The BLE driver class isn't exercised here (it needs bleak + a live
radio); only the synchronous, side-effect-free functions are.
"""
import pytest

from saint_server.host_peripherals.jbd_bms import (
    jbd_checksum,
    build_read_request,
    parse_basic_info_response,
    parse_cell_voltages_response,
    decode_protection_bits,
    decode_fet_status,
    _has_complete_frame,
    JBD_FRAME_START,
    JBD_FRAME_END,
    JBD_REG_BASIC_INFO,
    JBD_REG_CELL_VOLTAGES,
    JBD_STATUS_OK,
    JBD_STATUS_ERROR,
)


# ── frame builders (mirror the wire envelope) ──────────────────────

def _frame(register: int, status: int, data: bytes) -> bytes:
    length = len(data)
    crc = jbd_checksum(bytes([length]) + data)
    return (bytes([JBD_FRAME_START, register, status, length])
            + data
            + bytes([(crc >> 8) & 0xFF, crc & 0xFF, JBD_FRAME_END]))


def _basic_data(*, pack_mv=1320, current_10ma=150, remain_10mah=500,
                full_10mah=1000, cycles=42, protection=0x0003, soc=87,
                fet=0x03, cells=4, ntc=2, temps_raw=(2981, 2991)) -> bytes:
    d = bytearray(23 + 2 * ntc)

    def pu16(off, v):
        d[off] = (v >> 8) & 0xFF
        d[off + 1] = v & 0xFF

    pu16(0, pack_mv)
    pu16(2, current_10ma & 0xFFFF)   # signed → two's complement on the wire
    pu16(4, remain_10mah)
    pu16(6, full_10mah)
    pu16(8, cycles)
    pu16(16, protection)
    d[19] = soc
    d[20] = fet
    d[21] = cells
    d[22] = ntc
    for i, raw in enumerate(temps_raw[:ntc]):
        pu16(23 + i * 2, raw)
    return bytes(d)


# ── checksum + request builder (independent known-answers) ─────────

def test_jbd_checksum_known_values():
    assert jbd_checksum(b"") == 0
    assert jbd_checksum(bytes([1])) == 0xFFFF          # 0 - 1, 16-bit
    assert jbd_checksum(bytes([0x03, 0x00])) == 0xFFFD  # read-reg-3 payload


def test_build_read_request_basic_info():
    # 0xDD 0xA5 <reg> 0x00 <crc_hi> <crc_lo> 0x77, crc over [reg,0x00].
    assert build_read_request(JBD_REG_BASIC_INFO) == bytes(
        [0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77])


# ── basic-info parser ──────────────────────────────────────────────

def test_parse_basic_info_decodes_all_fields():
    frame = _frame(JBD_REG_BASIC_INFO, JBD_STATUS_OK, _basic_data())
    out = parse_basic_info_response(frame)
    assert out is not None
    assert out["pack_voltage"] == pytest.approx(13.2)
    assert out["current"] == pytest.approx(1.5)
    assert out["remain_cap"] == pytest.approx(5.0)
    assert out["full_cap"] == pytest.approx(10.0)
    assert out["cycles"] == 42
    assert out["soc"] == 87
    assert out["protection"] == 3
    assert out["fet_status"] == 3
    assert out["cell_count"] == 4
    assert out["temps"] == pytest.approx([25.0, 26.0])


def test_parse_basic_info_signed_current_discharge():
    # Negative current = discharging; must decode as signed.
    frame = _frame(JBD_REG_BASIC_INFO, JBD_STATUS_OK,
                   _basic_data(current_10ma=-150))
    out = parse_basic_info_response(frame)
    assert out["current"] == pytest.approx(-1.5)


@pytest.mark.parametrize("mangle, why", [
    (lambda f: f[:6], "too short"),
    (lambda f: bytes([0x00]) + f[1:], "bad start byte"),
    (lambda f: f[:-1] + bytes([0x00]), "bad end byte"),
    (lambda f: bytes([f[0], 0x99]) + f[2:], "wrong register"),
    (lambda f: bytes([f[0], f[1], JBD_STATUS_ERROR]) + f[3:], "error status"),
    (lambda f: f[:-2] + bytes([0xFF, f[-1]]), "corrupt checksum"),
])
def test_parse_basic_info_rejects_bad_frames(mangle, why):
    good = _frame(JBD_REG_BASIC_INFO, JBD_STATUS_OK, _basic_data())
    assert parse_basic_info_response(mangle(good)) is None, why


def test_parse_basic_info_rejects_short_data_section():
    # data_len < 23 → not a valid basic-info payload.
    frame = _frame(JBD_REG_BASIC_INFO, JBD_STATUS_OK, bytes(10))
    assert parse_basic_info_response(frame) is None


# ── cell-voltage parser ────────────────────────────────────────────

def test_parse_cell_voltages_decodes_mv_to_volts():
    cells_mv = [3300, 3310, 3290, 3305]
    data = bytearray()
    for mv in cells_mv:
        data += bytes([(mv >> 8) & 0xFF, mv & 0xFF])
    frame = _frame(JBD_REG_CELL_VOLTAGES, JBD_STATUS_OK, bytes(data))
    out = parse_cell_voltages_response(frame)
    assert out == pytest.approx([3.300, 3.310, 3.290, 3.305])


def test_parse_cell_voltages_rejects_odd_length():
    frame = _frame(JBD_REG_CELL_VOLTAGES, JBD_STATUS_OK, bytes(3))  # odd
    assert parse_cell_voltages_response(frame) is None


def test_parse_cell_voltages_rejects_wrong_register():
    frame = _frame(JBD_REG_BASIC_INFO, JBD_STATUS_OK, bytes([0x0C, 0xE4]))
    assert parse_cell_voltages_response(frame) is None


# ── bit/byte decoders ──────────────────────────────────────────────

def test_decode_protection_bits():
    assert decode_protection_bits(0) == []
    assert decode_protection_bits(0b1) == ["cell overvoltage"]
    assert decode_protection_bits(0b11) == ["cell overvoltage", "cell undervoltage"]
    assert decode_protection_bits(1 << 10) == ["short circuit"]
    # Reserved/unknown bits are surfaced, not dropped.
    assert decode_protection_bits(1 << 13) == ["fault bit 13"]


def test_decode_fet_status():
    assert decode_fet_status(0x00) == "both OFF"
    assert decode_fet_status(0x01) == "charge ON, discharge OFF"
    assert decode_fet_status(0x02) == "discharge ON, charge OFF"
    assert decode_fet_status(0x03) == "charge+discharge ON"


def test_has_complete_frame():
    assert _has_complete_frame(b"\xdd\x03") is False           # too short
    assert _has_complete_frame(b"\xdd\x03\x00\x04\x77") is True  # start + end
    assert _has_complete_frame(b"\x00\x03\x00\x77") is False     # bad start
