import csv
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException

HOST = "127.0.0.1"
PORT = 1502
DEVICE_ID = 1

# Virtual Modbus Interface v2: operational input-register contract.
IR_TEMP1 = 0
IR_TEMP2 = 1
IR_TEMP3 = 2
IR_ESP32_HEARTBEAT = 3
IR_SENSOR1_HEALTH = 4
IR_SENSOR2_HEALTH = 5
IR_SENSOR3_HEALTH = 6

# Test/commissioning-only holding register.
HR_FAULT_MODE = 0

INVALID = 65535

CHANNEL_HEALTHY = 0
CHANNEL_READ_FAILURE = 1
CHANNEL_OUT_OF_RANGE = 2
CHANNEL_STALE = 3
CHANNEL_STUCK = 4

NONE = 0
SENSOR1_DISCONNECTED = 1
SENSOR1_BIAS = 4
SENSOR1_STUCK = 6
COMMON_PROCESS_CHANGE = 10
ALL_SENSORS_DISCONNECTED = 17
SENSOR3_OUT_OF_RANGE = 20
SENSOR2_STALE_ACQUISITION = 22

POLL_INTERVAL_SECONDS = 0.35
STATE_TIMEOUT_SECONDS = 35.0
TRANSPORT_RETRIES = 8

_CLIENT = None
RESULTS = []


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def require_response(response, operation):
    if response.isError():
        raise AssertionError(f"{operation} failed: {response}")
    return response


def connect_client():
    client = ModbusTcpClient(HOST, port=PORT, timeout=3)
    if not client.connect():
        raise ConnectionError(
            f"Could not connect to Modbus TCP at {HOST}:{PORT}"
        )
    return client


def close_transport():
    global _CLIENT
    if _CLIENT is not None:
        _CLIENT.close()
        _CLIENT = None


def with_transport_retry(operation):
    global _CLIENT
    last_error = None

    for _ in range(TRANSPORT_RETRIES):
        try:
            if _CLIENT is None:
                _CLIENT = connect_client()
            return operation(_CLIENT)
        except (ConnectionError, ModbusException, OSError) as error:
            last_error = error
            close_transport()
            time.sleep(POLL_INTERVAL_SECONDS)

    raise ConnectionError("Modbus operation failed") from last_error


def write_fault_mode(mode):
    def operation(client):
        return require_response(
            client.write_register(
                HR_FAULT_MODE,
                mode,
                device_id=DEVICE_ID,
            ),
            f"write fault mode {mode}",
        )

    with_transport_retry(operation)


def read_snapshot():
    def operation(client):
        registers = require_response(
            client.read_input_registers(
                0,
                count=7,
                device_id=DEVICE_ID,
            ),
            "read v2 input registers",
        ).registers

        return {
            "temp1": registers[IR_TEMP1],
            "temp2": registers[IR_TEMP2],
            "temp3": registers[IR_TEMP3],
            "heartbeat": registers[IR_ESP32_HEARTBEAT],
            "sensor1_health": registers[IR_SENSOR1_HEALTH],
            "sensor2_health": registers[IR_SENSOR2_HEALTH],
            "sensor3_health": registers[IR_SENSOR3_HEALTH],
        }

    return with_transport_retry(operation)


def wait_for(description, predicate, timeout=STATE_TIMEOUT_SECONDS):
    deadline = time.time() + timeout
    last = None

    while time.time() < deadline:
        last = read_snapshot()
        if predicate(last):
            print(f"  PASS {description}: {last}")
            return last
        time.sleep(POLL_INTERVAL_SECONDS)

    raise AssertionError(
        f"Timeout waiting for {description}. Last snapshot: {last}"
    )


def all_healthy(snapshot):
    return (
        snapshot["sensor1_health"] == CHANNEL_HEALTHY
        and snapshot["sensor2_health"] == CHANNEL_HEALTHY
        and snapshot["sensor3_health"] == CHANNEL_HEALTHY
        and snapshot["temp1"] != INVALID
        and snapshot["temp2"] != INVALID
        and snapshot["temp3"] != INVALID
    )


def restore_normal():
    write_fault_mode(NONE)
    return wait_for(
        "recovery to healthy three-channel state",
        all_healthy,
    )


def test_normal_and_heartbeat():
    normal = restore_normal()
    first_heartbeat = normal["heartbeat"]

    later = wait_for(
        "heartbeat changes on a later acquisition cycle",
        lambda s: s["heartbeat"] != first_heartbeat,
    )

    return {
        "normal_snapshot": normal,
        "later_snapshot": later,
    }


def test_bias():
    baseline = restore_normal()
    write_fault_mode(SENSOR1_BIAS)

    biased = wait_for(
        "sensor 1 bias remains acquisition-healthy and numeric",
        lambda s: (
            all_healthy(s)
            and abs(s["temp1"] - s["temp2"]) >= 150
        ),
    )

    return {
        "baseline": baseline,
        "biased": biased,
        "note": (
            "Acquisition health remains HEALTHY by design; "
            "PLC owns disagreement/outlier interpretation."
        ),
    }


def test_sensor1_disconnect_and_recovery():
    restore_normal()
    write_fault_mode(SENSOR1_DISCONNECTED)

    failed = wait_for(
        "sensor 1 read failure",
        lambda s: (
            s["sensor1_health"] == CHANNEL_READ_FAILURE
            and s["temp1"] == INVALID
            and s["sensor2_health"] == CHANNEL_HEALTHY
            and s["sensor3_health"] == CHANNEL_HEALTHY
        ),
    )

    recovered = restore_normal()
    return {"failed": failed, "recovered": recovered}


def test_sensor3_out_of_range_and_recovery():
    restore_normal()
    write_fault_mode(SENSOR3_OUT_OF_RANGE)

    failed = wait_for(
        "sensor 3 out-of-range diagnostic",
        lambda s: (
            s["sensor3_health"] == CHANNEL_OUT_OF_RANGE
            and s["temp3"] == INVALID
        ),
    )

    recovered = restore_normal()
    return {"failed": failed, "recovered": recovered}


def test_sensor2_stale_and_recovery():
    restore_normal()
    write_fault_mode(SENSOR2_STALE_ACQUISITION)

    failed = wait_for(
        "sensor 2 stale-acquisition diagnostic",
        lambda s: (
            s["sensor2_health"] == CHANNEL_STALE
            and s["temp2"] == INVALID
        ),
    )

    recovered = restore_normal()
    return {"failed": failed, "recovered": recovered}


def test_sensor1_stuck_and_recovery():
    restore_normal()
    write_fault_mode(SENSOR1_STUCK)

    failed = wait_for(
        "sensor 1 stuck diagnostic",
        lambda s: (
            s["sensor1_health"] == CHANNEL_STUCK
            and s["temp1"] == INVALID
        ),
    )

    recovered = restore_normal()
    return {"failed": failed, "recovered": recovered}


def test_common_process_change():
    baseline = restore_normal()
    write_fault_mode(COMMON_PROCESS_CHANGE)

    changed = wait_for(
        "common process change keeps all three channels healthy",
        lambda s: (
            all_healthy(s)
            and s["temp1"] >= baseline["temp1"] + 50
            and s["temp2"] >= baseline["temp2"] + 50
            and s["temp3"] >= baseline["temp3"] + 50
        ),
    )

    return {
        "baseline": baseline,
        "changed": changed,
        "note": (
            "All channels moved together and remained HEALTHY; "
            "this guards against false stuck/fault classification "
            "during a genuine common process change."
        ),
    }


def test_all_sensors_disconnected_and_recovery():
    restore_normal()
    write_fault_mode(ALL_SENSORS_DISCONNECTED)

    failed = wait_for(
        "all three sensors read-failed",
        lambda s: (
            s["sensor1_health"] == CHANNEL_READ_FAILURE
            and s["sensor2_health"] == CHANNEL_READ_FAILURE
            and s["sensor3_health"] == CHANNEL_READ_FAILURE
            and s["temp1"] == INVALID
            and s["temp2"] == INVALID
            and s["temp3"] == INVALID
        ),
    )

    recovered = restore_normal()
    return {"failed": failed, "recovered": recovered}


def record_result(test_id, name, status, started, ended, details):
    RESULTS.append(
        {
            "test_id": test_id,
            "name": name,
            "status": status,
            "started_utc": started,
            "ended_utc": ended,
            "details": details,
        }
    )


def run_case(test_id, name, function):
    print(f"\n{test_id} — {name}")
    started = utc_now()

    try:
        details = function()
        ended = utc_now()
        record_result(
            test_id,
            name,
            "PASS",
            started,
            ended,
            json.dumps(details, sort_keys=True),
        )
        print(f"{test_id} RESULT: PASS")
        return True
    except Exception as error:
        ended = utc_now()
        record_result(
            test_id,
            name,
            "FAIL",
            started,
            ended,
            repr(error),
        )
        print(f"{test_id} RESULT: FAIL — {error}")

        try:
            restore_normal()
        except Exception as recovery_error:
            print(
                "  WARNING: automatic return to NONE/healthy state failed: "
                f"{recovery_error}"
            )
        return False


def write_reports():
    output_dir = Path("fat_results")
    output_dir.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = output_dir / f"virtual_automated_fat_{stamp}.csv"
    json_path = output_dir / f"virtual_automated_fat_{stamp}.json"

    fieldnames = [
        "test_id",
        "name",
        "status",
        "started_utc",
        "ended_utc",
        "details",
    ]

    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(RESULTS)

    with json_path.open("w", encoding="utf-8") as handle:
        json.dump(RESULTS, handle, indent=2)

    return csv_path, json_path


def main():
    print("FINAL VIRTUAL AUTOMATED FAT — acquisition/interface subset")
    print(f"Target: Modbus TCP {HOST}:{PORT}, device {DEVICE_ID}")
    print("Operational contract: 7 input registers; HR0 is test-only FAULT_MODE")
    print()
    print(
        "This runner automatically verifies the ESP32 acquisition/diagnostic "
        "and Modbus interface behaviors. Communication-loss and PLC trip/reset "
        "tests remain separate manual acceptance checks."
    )

    cases = [
        ("FAT-A01", "Normal operation and heartbeat", test_normal_and_heartbeat),
        ("FAT-A02", "Single-sensor bias remains acquisition-healthy", test_bias),
        (
            "FAT-A03",
            "Sensor 1 disconnect and recovery",
            test_sensor1_disconnect_and_recovery,
        ),
        (
            "FAT-A04",
            "Sensor 3 out-of-range and recovery",
            test_sensor3_out_of_range_and_recovery,
        ),
        (
            "FAT-A05",
            "Sensor 2 stale acquisition and recovery",
            test_sensor2_stale_and_recovery,
        ),
        (
            "FAT-A06",
            "Sensor 1 stuck diagnostic and recovery",
            test_sensor1_stuck_and_recovery,
        ),
        (
            "FAT-A07",
            "Common process change without false sensor fault",
            test_common_process_change,
        ),
        (
            "FAT-A08",
            "All sensors disconnected and recovery",
            test_all_sensors_disconnected_and_recovery,
        ),
    ]

    passed = 0

    try:
        for test_id, name, function in cases:
            if run_case(test_id, name, function):
                passed += 1
    finally:
        try:
            write_fault_mode(NONE)
        except Exception:
            pass
        close_transport()

    csv_path, json_path = write_reports()

    failed = len(cases) - passed
    print("\n" + "=" * 72)
    print(f"AUTOMATED FAT SUMMARY: {passed} PASS / {failed} FAIL")
    print(f"CSV evidence:  {csv_path}")
    print(f"JSON evidence: {json_path}")
    print("Manual outstanding: communication loss/recovery; high-temperature trip/reset.")
    print("=" * 72)

    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
