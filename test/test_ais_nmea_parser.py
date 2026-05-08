from aivn_sensorpack_ais.ais_nmea_parser import AisNmeaParser


def _checksum(body: str) -> str:
    value = 0
    for ch in body:
        value ^= ord(ch)
    return f"{value:02X}"


def _armored(value: int) -> str:
    code = value + 48
    if code > 87:
        code += 8
    return chr(code)


def _payload_from_bits(bits: str) -> tuple[str, int]:
    fill_bits = (6 - len(bits) % 6) % 6
    padded = bits + ("0" * fill_bits)
    payload = "".join(_armored(int(padded[i:i + 6], 2)) for i in range(0, len(padded), 6))
    return payload, fill_bits


def _uint(value: int, width: int) -> str:
    return f"{value:0{width}b}"[-width:]


def _int(value: int, width: int) -> str:
    if value < 0:
        value = (1 << width) + value
    return _uint(value, width)


def _sentence(payload: str, fill_bits: int) -> str:
    body = f"AIVDM,1,1,,A,{payload},{fill_bits}"
    return f"!{body}*{_checksum(body)}"


def test_type_1_position_report_decodes_lat_lon():
    bits = ""
    bits += _uint(1, 6)
    bits += _uint(0, 2)
    bits += _uint(440123456, 30)
    bits += _uint(0, 4)
    bits += _uint(128, 8)
    bits += _uint(123, 10)
    bits += _uint(0, 1)
    bits += _int(int(129.123456 * 600000), 28)
    bits += _int(int(35.123456 * 600000), 27)
    bits += _uint(456, 12)
    bits += _uint(90, 9)
    bits += _uint(12, 6)
    bits += "0" * 19
    payload, fill_bits = _payload_from_bits(bits)

    decoded = AisNmeaParser().parse_sentence(_sentence(payload, fill_bits))

    assert decoded is not None
    assert decoded.ais_message_id == 1
    assert decoded.mmsi == 440123456
    assert decoded.position_valid is True
    assert abs(decoded.lat - 35.123456) < 1e-5
    assert abs(decoded.lon - 129.123456) < 1e-5
    assert decoded.sog == 12.3
    assert decoded.cog == 45.6
    assert decoded.heading == 90
