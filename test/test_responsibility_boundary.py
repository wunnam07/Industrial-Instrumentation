import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
PLATFORMIO = (ROOT / "platformio.ini").read_text(encoding="utf-8")


class SimulatedResponsibilityBoundaryTests(unittest.TestCase):
    def test_active_environment_disables_physical_output(self):
        self.assertIn(
            "-DENABLE_PHYSICAL_HEATER_OUTPUT=0",
            PLATFORMIO,
        )

    def test_three_sensor_instrumentation_is_present(self):
        for sensor_number, gpio in ((1, 18), (2, 19), (3, 21)):
            self.assertRegex(
                SOURCE,
                rf"const int TEMP{sensor_number}_PIN\s*=\s*{gpio};",
            )
        for required_symbol in (
            "runDiagnostics",
            "detectStuckSensors",
            "IR_ESP32_HEARTBEAT",
            "IR_DIAGNOSTIC_STATUS",
            "IR_SENSOR1_HEALTH",
            "IR_SENSOR2_HEALTH",
            "IR_SENSOR3_HEALTH",
        ):
            self.assertIn(required_symbol, SOURCE)

    def test_process_control_symbols_are_absent(self):
        for forbidden_symbol in (
            "enum ProcessState",
            "updateProcessState(",
            "updateHeaterPermissive(",
            "tControl",
        ):
            self.assertNotIn(forbidden_symbol, SOURCE)
        for forbidden_identifier in ("T_ON", "T_OFF"):
            self.assertIsNone(
                re.search(
                    rf"\b{forbidden_identifier}\b",
                    SOURCE,
                )
            )

    def test_candidate_is_explicitly_non_authoritative(self):
        self.assertIn("IR_ESP32_TEMP_CANDIDATE", SOURCE)
        self.assertIn("diagnosticTempCandidate", SOURCE)
        self.assertIn(
            "PLC must select the authoritative process-control temperature",
            SOURCE,
        )

    def test_legacy_process_state_is_not_published(self):
        initialization = re.compile(
            r"mb\.addIreg\(\s*IR_LEGACY_PROCESS_STATE,\s*"
            r"MODBUS_INVALID_VALUE\s*\);",
            re.MULTILINE,
        )
        publication = re.compile(
            r"mb\.Ireg\(\s*IR_LEGACY_PROCESS_STATE,\s*"
            r"MODBUS_INVALID_VALUE\s*\);",
            re.MULTILINE,
        )
        self.assertRegex(SOURCE, initialization)
        self.assertRegex(SOURCE, publication)

    def test_simulated_path_forces_physical_status_false(self):
        for status in (
            "DI_PHYSICAL_OUTPUT_PERMISSIVE",
            "DI_PHYSICAL_HEATER_OUTPUT",
            "DI_PHYSICAL_PLC_COMMS_OK",
        ):
            self.assertIn(f"mb.Ists({status}, false);", SOURCE)


if __name__ == "__main__":
    unittest.main()
