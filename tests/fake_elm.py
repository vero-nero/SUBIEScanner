"""Minimal ELM327 simulator used by the tests (no hardware needed).

Default vehicle: a Ford on CAN 11-bit with an engine (7E8) and a
transmission (7E9) module. Responses are written the way a real ELM327
prints them with ATS0 (no spaces), with or without headers.
"""

from __future__ import annotations


def ford_can_responses() -> dict[str, list[str]]:
    """Command -> response lines with headers on (ATH1)."""
    return {
        "0100": ["7E806410098188013", "7E906410098000001"],
        "0120": ["7E806412000000000"],
        # Three stored codes from the engine (multi-frame), one from the TCM.
        "03": ["7E8100843030171", "7E82103000420AAAA", "7E904430107000000"],
        "07": ["7E8024700"],
        "0A": ["7E8037F0A11"],
        "0101": ["7E806410183076500"],
        "0902": ["7E8101449020131464D", "7E82143553948383547", "7E82241313132333435"],
        "0904": ["7E8037F0912"],
        "090A": ["7E8037F0912"],
        "020200": ["7E80542020001 71".replace(" ", "")],
        "020500": ["7E8044205005A"],
        "020C00": ["7E805420C001AF8"],
        "04": ["7E80144", "7E90144"],
        "010C": ["7E804410C1AF8"],
        "0105": ["7E8034105 5A".replace(" ", "")],
        # Multi-PID request (quick profile), engine answers multi-frame, TCM answers too
        "010104050C0D11": ["7E8101141018307 6500".replace(" ", ""), "7E82104800 55A0C1AF8".replace(" ", ""),
                           "7E903410D32", "7E822 0D321140AAAAAA".replace(" ", "")],
        # Ford module scan (physical addressing, keyed by ATSH header)
        "7E0:19028F": ["7E8075902FF01710009"],
        "760:19028F": ["76810 0B 59 02 FF 40 40 12".replace(" ", ""), "7682109C1008728AAAA"],
        "737:19028F": ["73F037F1911"],
        "737:1800FF00": ["73F05580193 18E0".replace(" ", "")],
        "760:14FFFFFF": ["7680154"],
    }


class FakeSerial:
    def __init__(self, responses: dict[str, list[str]] | None = None, protocol: str = "6") -> None:
        self.responses = responses if responses is not None else ford_can_responses()
        self.protocol = protocol
        self.headers = False
        self.target = "7DF"
        self.out = b""
        self.is_open = True
        self.sent: list[str] = []

    # pyserial API subset
    @property
    def in_waiting(self) -> int:
        return len(self.out)

    def read(self, size: int = 1) -> bytes:
        chunk, self.out = self.out[:size], self.out[size:]
        return chunk

    def reset_input_buffer(self) -> None:
        self.out = b""

    def reset_output_buffer(self) -> None:
        pass

    def flush(self) -> None:
        pass

    def close(self) -> None:
        self.is_open = False

    def write(self, data: bytes) -> None:
        command = data.decode("ascii").strip().upper().replace(" ", "")
        self.sent.append(command)
        self.out = (self.reply(command) + "\r\r>").encode("ascii")

    def reply(self, command: str) -> str:
        if command == "ATZ":
            return "\r\rELM327 v1.5"
        if command == "ATDPN":
            return "A" + self.protocol
        if command == "ATDP":
            return "AUTO, ISO 15765-4 (CAN 11/500)"
        if command.startswith("ATH"):
            self.headers = command == "ATH1"
            return "OK"
        if command.startswith("ATSH"):
            self.target = command[4:]
            return "OK"
        if command.startswith("AT"):
            return "OK"
        if self.target != "7DF":
            lines = self.responses.get(f"{self.target}:{command}")
        else:
            lines = self.responses.get(command)
        if not lines:
            return "NO DATA"
        if self.headers:
            return "\r".join(lines)
        # Headers off: strip the 3-char CAN id and PCI byte (single frames only).
        return "\r".join(line[5:] for line in lines if line[3] == "0")
