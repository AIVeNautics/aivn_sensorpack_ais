"""AIS 6-bit payload helpers."""

from __future__ import annotations


AIS_TEXT_TABLE = (
    "@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_"
    " !\"#$%&'()*+,-./0123456789:;<=>?"
)


def payload_char_to_value(ch: str) -> int:
    if len(ch) != 1:
        raise ValueError("AIS payload character must be one byte")
    value = ord(ch) - 48
    if value > 40:
        value -= 8
    if value < 0 or value > 63:
        raise ValueError(f"invalid AIS payload character: {ch!r}")
    return value


def payload_to_bits(payload: str, fill_bits: int = 0) -> str:
    if fill_bits < 0 or fill_bits > 5:
        raise ValueError(f"invalid AIS fill bits: {fill_bits}")
    bits = "".join(f"{payload_char_to_value(ch):06b}" for ch in payload)
    if fill_bits:
        if fill_bits > len(bits):
            raise ValueError("fill bits longer than payload")
        bits = bits[:-fill_bits]
    return bits


def get_uint(bits: str, start: int, length: int) -> int:
    end = start + length
    if start < 0 or length <= 0 or end > len(bits):
        raise ValueError(f"bit range [{start}:{end}] outside payload length {len(bits)}")
    return int(bits[start:end], 2)


def get_int(bits: str, start: int, length: int) -> int:
    value = get_uint(bits, start, length)
    sign_bit = 1 << (length - 1)
    if value & sign_bit:
        value -= 1 << length
    return value


def get_text(bits: str, start: int, length: int) -> str:
    end = start + length
    if end > len(bits):
        raise ValueError(f"text range [{start}:{end}] outside payload length {len(bits)}")
    chars = []
    for off in range(start, end, 6):
        chunk = bits[off:off + 6]
        if len(chunk) < 6:
            break
        chars.append(AIS_TEXT_TABLE[int(chunk, 2)])
    return "".join(chars).replace("@", "").strip()
