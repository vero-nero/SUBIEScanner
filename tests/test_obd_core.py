"""Run with:  py -m unittest discover tests"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import obd_core  # noqa: E402
from fake_elm import FakeSerial  # noqa: E402


def connected_elm(**kwargs) -> obd_core.ELM327:
    elm = obd_core.ELM327()
    elm.serial = FakeSerial(**kwargs)
    elm.initialise("A6")
    return elm


class FordCanTest(unittest.TestCase):
    def test_connect_and_discover(self):
        elm = connected_elm()
        self.assertTrue(elm.is_can)
        primary, supported = obd_core.discover_supported_pids(elm)
        self.assertEqual(primary, "7E8")
        self.assertIn(0x0C, supported["7E8"])
        info = obd_core.read_vehicle_info(elm)
        self.assertEqual(info["vin"], "1FMCU9H85GA112345")
        self.assertEqual(info["make"], "Ford")

    def test_dtcs_multi_frame_count_byte_and_modules(self):
        elm = connected_elm()
        dtcs, notes = obd_core.read_generic_dtcs(elm, "Ford")
        stored = [(d.code, d.module) for d in dtcs if d.status == "Stored"]
        self.assertEqual(stored, [("P0171", "Engine (ECM/PCM)"), ("P0300", "Engine (ECM/PCM)"),
                                  ("P0420", "Engine (ECM/PCM)"), ("P0700", "Transmission (TCM)")])
        self.assertFalse([d for d in dtcs if d.status == "Pending"])
        self.assertTrue(any("Permanent" in note for note in notes))

    def test_freeze_frame_skips_frame_byte(self):
        elm = connected_elm()
        lines = obd_core.read_freeze_frame(elm, [0x05, 0x0C], "7E8", "Ford")
        self.assertIn("Triggered by P0171", lines[1])
        self.assertTrue(any("Engine coolant temperature: 50 °C" in line for line in lines))
        self.assertTrue(any("Engine RPM: 1726 rpm" in line for line in lines))

    def test_readiness(self):
        elm = connected_elm()
        status = obd_core.read_monitor_status(elm)
        text = obd_core.format_readiness(status)
        self.assertIn("MIL ON, 3 confirmed", text)
        self.assertIn("READY      Catalyst", text)


class FordModuleScanTest(unittest.TestCase):
    def test_uds_and_kwp_modules(self):
        import ford
        elm = connected_elm()
        dtcs, results = ford.scan_modules(elm)
        found = {(d.code, d.module.split()[0], d.status) for d in dtcs}
        self.assertEqual(found, {("P0171", "PCM", "Active, Stored"), ("C0040", "ABS", "Active, Stored"),
                                 ("U0100", "ABS", "Stored"), ("B1318", "RCM", "Stored")})
        responded = {r.short: r.protocol for r in results if r.responded}
        self.assertEqual(responded, {"PCM": "UDS", "ABS": "UDS", "RCM": "KWP"})
        self.assertEqual(elm.serial.sent[-1], "ATSH7DF")  # broadcast header restored
        self.assertIn("ABS: cleared", ford.clear_module_dtcs(elm, results))


class KLineTest(unittest.TestCase):
    def test_kline_dtcs_have_no_count_byte(self):
        responses = {"0100": ["7E806410098188013"], "03": ["7E8074301330000000000"]}
        elm = connected_elm(responses=responses, protocol="3")
        self.assertFalse(elm.is_can)
        dtcs, _ = obd_core.read_generic_dtcs(elm)
        self.assertEqual([d.code for d in dtcs if d.status == "Stored"], ["P0133"])


if __name__ == "__main__":
    unittest.main()
