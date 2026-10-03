import csv
import json
import time
from datetime import datetime
from pathlib import Path

from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException

HOST = "127.0.0.1"
PORT = 1502
DEVICE_ID = 1

# Frozen Virtual Modbus Interface v2.
IR_TEMP1 = 0
IR_TEMP2 = 1
IR_TEMP3 = 2
IR_ESP32_HEARTBEAT = 3
IR_SENSOR1_HEALTH = 4
IR_SENSOR2_HEALTH = 5
IR_SENSOR3_HEALTH = 6
HR_FAULT_MODE = 0

INVALID = 65535

CHANNEL_HEALTHY = 0
CHANNEL_READ_FAILURE = 1
CHANNEL_OUT_OF_RANGE = 2
CHANNEL_STALE = 3
CHANNEL_STUCK = 4
CHANNEL_RECOVERING = 5
CHANNEL_NOT_READY = 6

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
SCENARIO_HOLD_SECONDS = 4.0

_CLIENT = None
RESULTS = []
SNAPSHOTS = []


def require_response(response, operation):
    if response.isError():
        raise AssertionError(f"{operation} failed: {response}")
    return response


def connect_client():
    client = ModbusTcpClient(HOST, port=PORT, timeout=3)
    if not client.connect():
        raise ConnectionError(f"Could not connect to Modbus TCP at {HOST}:{PORT}")
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
            client.write_register(HR_FAULT_MODE, mode, device_id=DEVICE_ID),
            f"write fault mode {mode}",
        )
    with_transport_retry(operation)


def read_snapshot():
    def operation(client):
        registers = require_response(
            client.read_input_registers(0, count=7, device_id=DEVICE_ID),
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


def record_snapshot(test_id, phase, snapshot):
    row = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "test_id": test_id,
        "phase": phase,
        **snapshot,
    }
    SNAPSHOTS.append(row)


def wait_for(test_id, description, predicate, timeout=STATE_TIMEOUT_SECONDS):
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        last = read_snapshot()
        if predicate(last):
            record_snapshot(test_id, description, last)
            print(f"PASS {test_id} - {description}: {last}")
            return last
        time.sleep(POLL_INTERVAL_SECONDS)
    raise AssertionError(f"Timeout waiting for {description}. Last snapshot: {last}")


def all_healthy(s):
    return (
        s["sensor1_health"] == CHANNEL_HEALTHY
        and s["sensor2_health"] == CHANNEL_HEALTHY
        and s["sensor3_health"] == CHANNEL_HEALTHY
        and s["temp1"] != INVALID
        and s["temp2"] != INVALID
        and s["temp3"] != INVALID
    )


def restore_normal(test_id):
    write_fault_mode(NONE)
    return wait_for(test_id, "recovery to healthy three-channel state", all_healthy)


def run_case(test_id, name, fn, plc_expectation):
    print("\n" + "=" * 72)
    print(f"{test_id} - {name}")
    print("PLC/TIA observation expected:")
    print(plc_expectation)
    print("=" * 72)
    started = datetime.now().isoformat(timespec="seconds")
    try:
        fn(test_id)
        result = "PASS"
        detail = "Automated ESP32/Modbus acceptance checks passed"
        print(f"PASS {test_id} - {name}")
    except Exception as exc:
        result = "FAIL"
        detail = str(exc)
        print(f"FAIL {test_id} - {name}: {exc}")
    RESULTS.append({
        "test_id": test_id,
        "test_name": name,
        "automated_result": result,
        "detail": detail,
        "plc_result": "OBSERVE_IN_TIA",
        "plc_expectation": plc_expectation.replace("\n", " | "),
        "started": started,
        "finished": datetime.now().isoformat(timespec="seconds"),
    })
    return result == "PASS"


def fat_a01_normal_heartbeat(test_id):
    normal = restore_normal(test_id)
    hb = normal["heartbeat"]
    wait_for(test_id, "heartbeat changes", lambda s: all_healthy(s) and s["heartbeat"] != hb)
    time.sleep(SCENARIO_HOLD_SECONDS)


def fat_a02_bias_disagreement(test_id):
    restore_normal(test_id)
    write_fault_mode(SENSOR1_BIAS)
    wait_for(
        test_id,
        "sensor 1 biased while all acquisition health remains healthy",
        lambda s: all_healthy(s) and abs(s["temp1"] - s["temp2"]) >= 150,
    )
    time.sleep(SCENARIO_HOLD_SECONDS)
    restore_normal(test_id)


def fat_a03_disconnect_recovery(test_id):
    restore_normal(test_id)
    write_fault_mode(SENSOR1_DISCONNECTED)
    wait_for(
        test_id,
        "sensor 1 read failure",
        lambda s: (
            s["sensor1_health"] == CHANNEL_READ_FAILURE
            and s["temp1"] == INVALID
            and s["sensor2_health"] == CHANNEL_HEALTHY
            and s["sensor3_health"] == CHANNEL_HEALTHY
        ),
    )
    time.sleep(SCENARIO_HOLD_SECONDS)
    restore_normal(test_id)


def fat_a04_out_of_range_recovery(test_id):
    restore_normal(test_id)
    write_fault_mode(SENSOR3_OUT_OF_RANGE)
    wait_for(
        test_id,
        "sensor 3 out-of-range",
        lambda s: s["sensor3_health"] == CHANNEL_OUT_OF_RANGE and s["temp3"] == INVALID,
    )
    time.sleep(SCENARIO_HOLD_SECONDS)
    restore_normal(test_id)


def fat_a05_stale_recovery(test_id):
    restore_normal(test_id)
    write_fault_mode(SENSOR2_STALE_ACQUISITION)
    wait_for(
        test_id,
        "sensor 2 stale",
        lambda s: s["sensor2_health"] == CHANNEL_STALE and s["temp2"] == INVALID,
    )
    time.sleep(SCENARIO_HOLD_SECONDS)
    restore_normal(test_id)


def fat_a06_stuck_recovery(test_id):
    restore_normal(test_id)
    write_fault_mode(SENSOR1_STUCK)
    wait_for(
        test_id,
        "sensor 1 stuck diagnostic",
        lambda s: s["sensor1_health"] == CHANNEL_STUCK and s["temp1"] == INVALID,
    )
    time.sleep(SCENARIO_HOLD_SECONDS)
    restore_normal(test_id)


def fat_a07_common_process_change(test_id):
    baseline = restore_normal(test_id)
    write_fault_mode(COMMON_PROCESS_CHANGE)
    wait_for(
        test_id,
        "common process change keeps all channels healthy and coherent",
        lambda s: (
            all_healthy(s)
            and s["temp1"] >= baseline["temp1"] + 50
            and s["temp2"] >= baseline["temp2"] + 50
            and s["temp3"] >= baseline["temp3"] + 50
            and max(s["temp1"], s["temp2"], s["temp3"]) - min(s["temp1"], s["temp2"], s["temp3"]) <= 100
        ),
    )
    time.sleep(SCENARIO_HOLD_SECONDS)
    restore_normal(test_id)


def fat_a08_all_sensors_failed_recovery(test_id):
    restore_normal(test_id)
    write_fault_mode(ALL_SENSORS_DISCONNECTED)
    wait_for(
        test_id,
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
    time.sleep(SCENARIO_HOLD_SECONDS)
    restore_normal(test_id)


def write_reports():
    evidence_dir = Path("fat_evidence")
    evidence_dir.mkdir(exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = evidence_dir / f"virtual_fat_automated_{stamp}.csv"
    json_path = evidence_dir / f"virtual_fat_snapshots_{stamp}.json"

    with csv_path.open("w", newline="", encoding="utf-8") as fh:
        fields = [
            "test_id", "test_name", "automated_result", "detail",
            "plc_result", "plc_expectation", "started", "finished",
        ]
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        writer.writerows(RESULTS)

    with json_path.open("w", encoding="utf-8") as fh:
        json.dump(SNAPSHOTS, fh, indent=2)

    return csv_path, json_path


def main():
    print("Final Virtual FAT - Automated Sensor/Modbus Campaign")
    print(f"Target: {HOST}:{PORT}, device {DEVICE_ID}")
    print("Manual FAT items intentionally excluded: communication-loss/recovery and trip/reset.")
    print("Keep TIA Portal online if you want to observe the PLC expectations printed for each case.")

    cases = [
        (
            "FAT-A01",
            "Normal acquisition and heartbeat",
            fat_a01_normal_heartbeat,
            "CommsOK = TRUE; trusted temperature valid; TEMP_QUALITY healthy; no trip.",
        ),
        (
            "FAT-A02",
            "Single-sensor bias / disagreement",
            fat_a02_bias_disagreement,
            "Sensor 1 remains acquisition-healthy; PLC detects disagreement and selects the agreeing pair (Sensors 2 and 3).",
        ),
        (
            "FAT-A03",
            "Single-sensor disconnect and recovery",
            fat_a03_disconnect_recovery,
            "Sensor 1 becomes unusable; Sensors 2 and 3 retain a valid trusted temperature; three-channel service returns after recovery qualification.",
        ),
        (
            "FAT-A04",
            "Out-of-range channel and recovery",
            fat_a04_out_of_range_recovery,
            "Sensor 3 becomes unusable; PLC continues from the remaining valid channels when agreement permits.",
        ),
        (
            "FAT-A05",
            "Stale channel and recovery",
            fat_a05_stale_recovery,
            "Sensor 2 becomes unusable due to acquisition health; PLC does not use it until recovery completes.",
        ),
        (
            "FAT-A06",
            "Stuck-channel diagnostic and recovery",
            fat_a06_stuck_recovery,
            "Sensor 1 becomes unusable after STUCK diagnosis; PLC continues from usable corroborating channels.",
        ),
        (
            "FAT-A07",
            "Common process change",
            fat_a07_common_process_change,
            "All channels move together and remain usable; PLC treats this as a real process change rather than an instrument disagreement.",
        ),
        (
            "FAT-A08",
            "All sensor channels failed and recovery",
            fat_a08_all_sensors_failed_recovery,
            "Trusted temperature becomes invalid / temperature quality unacceptable; heater permissive should be removed until usable measurements recover.",
        ),
    ]

    overall = True
    try:
        for case in cases:
            overall = run_case(*case) and overall
    finally:
        try:
            write_fault_mode(NONE)
        except Exception:
            pass
        close_transport()

    csv_path, json_path = write_reports()

    print("\n" + "=" * 72)
    print("AUTOMATED FAT SUMMARY")
    for row in RESULTS:
        print(f"{row['test_id']} {row['automated_result']:>4} - {row['test_name']}")
    print(f"CSV report:  {csv_path}")
    print(f"JSON evidence: {json_path}")
    print("Manual FAT still required separately: communication loss/recovery; trip/reset.")
    print("=" * 72)

    if not overall:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
