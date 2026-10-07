"""Ford enhanced diagnostics over HS-CAN (OBD pins 6/14, 500 kbaud, 11-bit).

Reads trouble codes from the individual modules (ABS, airbag, body, …)
with UDS ReadDTCInformation (19 02). Older Ford modules that only speak
KWP2000-on-CAN are read with ReadDTCByStatus (18 00 FF 00) instead.

Modules that live on Ford's medium-speed CAN (MS-CAN, pins 3/11) cannot
be reached by a standard ELM327; they simply show as "no response".
"""

from __future__ import annotations

from dataclasses import dataclass

from dtc_codes import describe_dtc
from obd_core import DTC, ELM327, decode_dtc_pair, describe_nrc

# (request id, short name, description). Response id = request id + 8.
FORD_MODULES: list[tuple[int, str, str]] = [
    (0x7E0, "PCM", "Powertrain control module"),
    (0x7E1, "TCM", "Transmission control module"),
    (0x760, "ABS", "Anti-lock brake / stability control"),
    (0x737, "RCM", "Restraints (airbag) control module"),
    (0x730, "PSCM", "Power steering control module"),
    (0x720, "IPC", "Instrument panel cluster"),
    (0x726, "BCM", "Body control module"),
    (0x716, "GWM", "Gateway module"),
    (0x7D0, "APIM", "SYNC / infotainment"),
    (0x733, "HVAC", "Climate control module"),
    (0x736, "PAM", "Parking aid module"),
    (0x727, "ACM", "Audio control module"),
    (0x724, "SCCM", "Steering column control module"),
    (0x706, "IPMA", "Image processing module (camera)"),
    (0x764, "CCM", "Cruise control (radar) module"),
    (0x740, "DDM", "Driver door module"),
    (0x741, "PDM", "Passenger door module"),
]

# ISO 14229 / SAE J2012 failure type bytes (the most common ones).
FAILURE_TYPES = {
    0x00: "no sub-type information",
    0x01: "general electrical failure",
    0x07: "mechanical failure",
    0x08: "bus signal / message failure",
    0x09: "component failure",
    0x11: "circuit short to ground",
    0x12: "circuit short to battery",
    0x13: "circuit open",
    0x16: "circuit voltage below threshold",
    0x17: "circuit voltage above threshold",
    0x1A: "circuit resistance below threshold",
    0x1B: "circuit resistance above threshold",
    0x1C: "circuit voltage out of range",
    0x1D: "circuit current out of range",
    0x29: "signal invalid",
    0x2F: "signal erratic",
    0x31: "no signal",
    0x49: "internal electronic failure",
    0x4B: "over temperature",
    0x54: "missing calibration",
    0x55: "not configured",
    0x62: "signal compare failure",
    0x64: "signal plausibility failure",
    0x68: "event information",
    0x71: "actuator stuck",
    0x81: "invalid serial data received",
    0x86: "signal invalid",
    0x87: "missing message",
    0x88: "bus off",
    0x92: "performance or incorrect operation",
    0x93: "no operation",
    0x96: "component internal failure",
}

STATUS_MASK = 0x8F  # failed now / this cycle, pending, confirmed, warning lamp


def uds_status(status: int) -> str:
    labels = []
    if status & 0x01:
        labels.append("Active")
    if status & 0x08:
        labels.append("Stored")
    if status & 0x04:
        labels.append("Pending")
    if not labels:
        labels.append("History")
    if status & 0x80:
        labels.append("lamp on")
    return ", ".join(labels)


@dataclass
class ModuleResult:
    short: str
    name: str
    request_id: int
    responded: bool = False
    protocol: str = ""      # "UDS" or "KWP"
    note: str = ""


def parse_uds_dtcs(data: list[int]) -> list[tuple[int, int, int, int]]:
    """59 02 <availability mask> then 4 bytes per DTC: high, middle, failure type, status."""
    body = data[3:]
    return [tuple(body[i:i + 4]) for i in range(0, len(body) - 3, 4)]


def parse_kwp_dtcs(data: list[int]) -> list[tuple[int, int, int]]:
    """58 <count> then 3 bytes per DTC: high, low, status."""
    count = data[1] if len(data) > 1 else 0
    body = data[2:2 + 3 * count]
    return [tuple(body[i:i + 3]) for i in range(0, len(body) - 2, 3)]


class _PhysicalAddressing:
    """Temporarily talk to one module instead of the OBD broadcast address."""

    def __init__(self, elm: ELM327) -> None:
        self.elm = elm

    def target(self, request_id: int) -> None:
        response_id = request_id + 8
        for command in (f"ATSH{request_id:03X}", f"ATCRA{response_id:03X}",
                        f"ATFCSH{request_id:03X}", "ATFCSD300000", "ATFCSM1"):
            self.elm.send(command, 1.0)

    def __enter__(self) -> "_PhysicalAddressing":
        return self

    def __exit__(self, *_exc) -> None:
        for command in ("ATCRA", "ATFCSM0", "ATSH7DF"):
            try:
                self.elm.send(command, 1.0)
            except Exception:
                pass


def require_can11(elm: ELM327) -> None:
    if not elm.is_can or elm.header_len != 3:
        raise RuntimeError("The Ford module scan needs an 11-bit CAN connection "
                           f"(connected protocol: {elm.protocol_name}).")


def scan_modules(elm: ELM327, make: str = "Ford",
                 progress=None) -> tuple[list[DTC], list[ModuleResult]]:
    require_can11(elm)
    dtcs: list[DTC] = []
    results: list[ModuleResult] = []

    with _PhysicalAddressing(elm) as bus:
        for request_id, short, name in FORD_MODULES:
            result = ModuleResult(short, name, request_id)
            results.append(result)
            if progress:
                progress(f"Scanning {short}…")
            bus.target(request_id)
            address = f"{request_id + 8:03X}"
            module = f"{short} – {name}"

            messages = [m for m in elm.request(f"1902{STATUS_MASK:02X}", 2.0) if m.address == address]
            if not messages:
                continue
            result.responded = True
            message = messages[0]

            if message.data[:2] == [0x59, 0x02]:
                result.protocol = "UDS"
                for high, middle, ftb, status in parse_uds_dtcs(message.data):
                    code = decode_dtc_pair(high, middle)
                    if not code:
                        continue
                    detail = f"{code}-{ftb:02X} {FAILURE_TYPES.get(ftb, 'failure type')}; status 0x{status:02X}"
                    dtcs.append(DTC(code, uds_status(status), module, describe_dtc(code, make), detail))
                continue

            if message.negative and message.nrc in (0x11, 0x12):
                kwp = [m for m in elm.request("1800FF00", 2.0) if m.address == address]
                if kwp and kwp[0].data[:1] == [0x58]:
                    result.protocol = "KWP"
                    for high, low, status in parse_kwp_dtcs(kwp[0].data):
                        code = decode_dtc_pair(high, low)
                        if code:
                            dtcs.append(DTC(code, "Stored", module, describe_dtc(code, make),
                                            f"KWP status 0x{status:02X}"))
                    continue
            result.note = describe_nrc(message.nrc) if message.negative else "unexpected answer"

    return dtcs, results


def clear_module_dtcs(elm: ELM327, modules: list[ModuleResult]) -> list[str]:
    """ClearDiagnosticInformation (14 FF FF FF) on every module that answered a scan."""
    require_can11(elm)
    lines = []
    with _PhysicalAddressing(elm) as bus:
        for result in modules:
            if not result.responded:
                continue
            bus.target(result.request_id)
            address = f"{result.request_id + 8:03X}"
            command = "14FFFFFF" if result.protocol != "KWP" else "14FF00"
            messages = [m for m in elm.request(command, 4.0) if m.address == address]
            if messages and messages[0].data[:1] == [0x54]:
                lines.append(f"{result.short}: cleared")
            elif messages and messages[0].negative:
                lines.append(f"{result.short}: refused – {describe_nrc(messages[0].nrc)}")
            else:
                lines.append(f"{result.short}: no confirmation")
    return lines


def format_module_summary(results: list[ModuleResult], dtcs: list[DTC]) -> str:
    lines = ["FORD MODULE SCAN (HS-CAN)", ""]
    for result in results:
        if result.responded:
            count = sum(1 for dtc in dtcs if dtc.module.startswith(result.short + " "))
            state = f"{count} code(s)" if not result.note else result.note
            lines.append(f"  {result.short:<5} {result.name:<38} {result.protocol or '—':<4} {state}")
    missing = [r.short for r in results if not r.responded]
    if missing:
        lines += ["", "No response (not fitted, or on MS-CAN which a standard ELM327 cannot reach): "
                  + ", ".join(missing)]
    return "\n".join(lines)
