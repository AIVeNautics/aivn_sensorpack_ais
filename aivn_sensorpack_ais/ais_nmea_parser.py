"""AIS NMEA 0183 parser for !AIVDM/!AIVDO sentences."""

from __future__ import annotations

from dataclasses import dataclass, field
import time
from typing import Callable, Dict, Optional, Tuple

from .ais_sixbit import get_int, get_text, get_uint, payload_to_bits


AIS_UNAVAILABLE_HEADING = 511
AIS_UNKNOWN_NAV_STATUS = 15


@dataclass
class AisDecoded:
    ais_message_id: int
    mmsi: int
    ship_id: str
    ship_name: str = ""
    call_sign: str = ""
    ship_type: int = 0
    navigation_status: int = AIS_UNKNOWN_NAV_STATUS
    lat: float = 0.0
    lon: float = 0.0
    sog: float = 0.0
    cog: float = 0.0
    heading: int = AIS_UNAVAILABLE_HEADING
    position_valid: bool = False
    static_valid: bool = False
    original_sentence: str = ""


@dataclass
class _FragmentAssembly:
    total: int
    fill_bits: int = 0
    first_seen: float = field(default_factory=time.monotonic)
    payloads: Dict[int, str] = field(default_factory=dict)
    sentences: Dict[int, str] = field(default_factory=dict)


class AisNmeaParser:
    def __init__(
        self,
        checksum_required: bool = True,
        fragment_ttl_sec: float = 8.0,
        debug_cb: Optional[Callable[[str], None]] = None,
    ):
        self.checksum_required = checksum_required
        self.fragment_ttl_sec = fragment_ttl_sec
        self.debug_cb = debug_cb
        self._fragments: Dict[Tuple[str, str, str], _FragmentAssembly] = {}

    def parse_sentence(self, sentence: str) -> Optional[AisDecoded]:
        s = (sentence or "").strip()
        if not s:
            return None
        if not (s.startswith("!AIVDM") or s.startswith("!AIVDO")):
            return None

        body, checksum = self._split_checksum(s)
        if self.checksum_required and not self._checksum_ok(body, checksum):
            raise ValueError(f"AIS checksum mismatch: {s}")

        parts = body.split(",")
        if len(parts) < 7:
            raise ValueError(f"AIS sentence has too few fields: {s}")

        sentence_id = parts[0]
        total = int(parts[1])
        number = int(parts[2])
        sequence_id = parts[3]
        channel = parts[4]
        payload = parts[5]
        fill_bits = int(parts[6])

        if total <= 0 or number <= 0 or number > total:
            raise ValueError(f"invalid AIS fragment numbering: {s}")

        self._prune_fragments()

        if total == 1:
            return self.decode_payload(payload, fill_bits, s)

        key = (sentence_id, sequence_id, channel)
        assembly = self._fragments.get(key)
        if assembly is None or assembly.total != total:
            assembly = _FragmentAssembly(total=total)
            self._fragments[key] = assembly
            self._debug(
                "fragment assembly started: "
                f"key={key} total={total} sentence={sentence_id}"
            )

        assembly.payloads[number] = payload
        assembly.sentences[number] = s
        self._debug(
            "fragment received: "
            f"key={key} part={number}/{total} "
            f"collected={len(assembly.payloads)}/{total}"
        )
        if number == total:
            assembly.fill_bits = fill_bits

        if len(assembly.payloads) != total:
            self._debug(
                "fragment assembly waiting: "
                f"key={key} missing={total - len(assembly.payloads)}"
            )
            return None

        combined_payload = "".join(assembly.payloads[i] for i in range(1, total + 1))
        combined_sentence = "\n".join(assembly.sentences[i] for i in range(1, total + 1))
        del self._fragments[key]
        self._debug(
            "fragment assembly completed: "
            f"key={key} total={total} payload_len={len(combined_payload)}"
        )
        return self.decode_payload(combined_payload, assembly.fill_bits, combined_sentence)

    def decode_payload(self, payload: str, fill_bits: int, original_sentence: str = "") -> Optional[AisDecoded]:
        bits = payload_to_bits(payload, fill_bits)
        if len(bits) < 38:
            raise ValueError("AIS payload too short")

        msg_id = get_uint(bits, 0, 6)
        mmsi = get_uint(bits, 8, 30)
        ship_id = str(mmsi)

        if msg_id in (1, 2, 3):
            return self._decode_position_class_a(bits, msg_id, mmsi, ship_id, original_sentence)
        if msg_id == 18:
            return self._decode_position_class_b(bits, msg_id, mmsi, ship_id, original_sentence)
        if msg_id == 19:
            return self._decode_position_class_b_extended(bits, msg_id, mmsi, ship_id, original_sentence)
        if msg_id == 5:
            return self._decode_static_type_5(bits, msg_id, mmsi, ship_id, original_sentence)
        if msg_id == 24:
            return self._decode_static_type_24(bits, msg_id, mmsi, ship_id, original_sentence)
        self._debug(f"unsupported AIS message type ignored: msg_id={msg_id} mmsi={mmsi}")
        return None

    def _decode_position_class_a(
        self, bits: str, msg_id: int, mmsi: int, ship_id: str, original_sentence: str
    ) -> AisDecoded:
        sog_raw = get_uint(bits, 50, 10)
        lon_raw = get_int(bits, 61, 28)
        lat_raw = get_int(bits, 89, 27)
        cog_raw = get_uint(bits, 116, 12)
        heading_raw = get_uint(bits, 128, 9)
        lat = lat_raw / 600000.0
        lon = lon_raw / 600000.0
        return AisDecoded(
            ais_message_id=msg_id,
            mmsi=mmsi,
            ship_id=ship_id,
            navigation_status=get_uint(bits, 38, 4),
            lat=lat,
            lon=lon,
            sog=0.0 if sog_raw == 1023 else sog_raw / 10.0,
            cog=0.0 if cog_raw == 3600 else cog_raw / 10.0,
            heading=heading_raw,
            position_valid=self._position_valid(lat, lon),
            original_sentence=original_sentence,
        )

    def _decode_position_class_b(
        self, bits: str, msg_id: int, mmsi: int, ship_id: str, original_sentence: str
    ) -> AisDecoded:
        sog_raw = get_uint(bits, 46, 10)
        lon_raw = get_int(bits, 57, 28)
        lat_raw = get_int(bits, 85, 27)
        cog_raw = get_uint(bits, 112, 12)
        heading_raw = get_uint(bits, 124, 9)
        lat = lat_raw / 600000.0
        lon = lon_raw / 600000.0
        return AisDecoded(
            ais_message_id=msg_id,
            mmsi=mmsi,
            ship_id=ship_id,
            lat=lat,
            lon=lon,
            sog=0.0 if sog_raw == 1023 else sog_raw / 10.0,
            cog=0.0 if cog_raw == 3600 else cog_raw / 10.0,
            heading=heading_raw,
            position_valid=self._position_valid(lat, lon),
            original_sentence=original_sentence,
        )

    def _decode_position_class_b_extended(
        self, bits: str, msg_id: int, mmsi: int, ship_id: str, original_sentence: str
    ) -> AisDecoded:
        decoded = self._decode_position_class_b(bits, msg_id, mmsi, ship_id, original_sentence)
        if len(bits) >= 263:
            decoded.ship_name = get_text(bits, 143, 120)
            decoded.ship_type = get_uint(bits, 263, 8) if len(bits) >= 271 else 0
            decoded.static_valid = bool(decoded.ship_name or decoded.ship_type)
        return decoded

    def _decode_static_type_5(
        self, bits: str, msg_id: int, mmsi: int, ship_id: str, original_sentence: str
    ) -> AisDecoded:
        if len(bits) < 240:
            raise ValueError("AIS type 5 payload too short")
        return AisDecoded(
            ais_message_id=msg_id,
            mmsi=mmsi,
            ship_id=ship_id,
            ship_name=get_text(bits, 112, 120),
            call_sign=get_text(bits, 70, 42),
            ship_type=get_uint(bits, 232, 8),
            static_valid=True,
            original_sentence=original_sentence,
        )

    def _decode_static_type_24(
        self, bits: str, msg_id: int, mmsi: int, ship_id: str, original_sentence: str
    ) -> AisDecoded:
        if len(bits) < 40:
            raise ValueError("AIS type 24 payload too short")
        part_number = get_uint(bits, 38, 2)
        decoded = AisDecoded(
            ais_message_id=msg_id,
            mmsi=mmsi,
            ship_id=ship_id,
            static_valid=True,
            original_sentence=original_sentence,
        )
        if part_number == 0 and len(bits) >= 160:
            decoded.ship_name = get_text(bits, 40, 120)
        elif part_number == 1 and len(bits) >= 132:
            decoded.ship_type = get_uint(bits, 40, 8)
            decoded.call_sign = get_text(bits, 90, 42)
        return decoded

    def _split_checksum(self, sentence: str) -> Tuple[str, str]:
        star = sentence.rfind("*")
        if star == -1:
            if self.checksum_required:
                raise ValueError(f"AIS sentence missing checksum: {sentence}")
            return sentence[1:], ""
        checksum = sentence[star + 1:star + 3]
        if len(checksum) != 2:
            raise ValueError(f"AIS sentence checksum is incomplete: {sentence}")
        return sentence[1:star], checksum

    @staticmethod
    def _checksum_ok(body: str, checksum: str) -> bool:
        calc = 0
        for ch in body:
            calc ^= ord(ch)
        try:
            expected = int(checksum, 16)
        except ValueError:
            return False
        return calc == expected

    @staticmethod
    def _position_valid(lat: float, lon: float) -> bool:
        return -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0

    def _prune_fragments(self) -> None:
        now = time.monotonic()
        stale_keys = [
            key for key, assembly in self._fragments.items()
            if now - assembly.first_seen > self.fragment_ttl_sec
        ]
        for key in stale_keys:
            self._debug(f"fragment assembly expired: key={key}")
            del self._fragments[key]

    def _debug(self, message: str) -> None:
        if self.debug_cb is not None:
            self.debug_cb(message)
