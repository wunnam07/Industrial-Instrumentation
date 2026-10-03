import time

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
CHANNEL_RECOVERING = 5
CHANNEL_NOT_READY = 6

NONE = 0
SENSOR1_DISCONNECTED = 1
SENSOR2_DISCONNECTED = 2
BOTH_SENSORS_DISCONNECTED = 3
SENSOR1_BIAS = 4
SENSOR2_BIAS = 5
SENSOR1_STUCK = 6
SENSOR2_STUCK = 7
TEMPORARY_DISAGREEMENT = 8
PERSISTENT_DISAGREEMENT = 9
COMMON_PROCESS_CHANGE = 10
SENSOR1_DRIFT = 11
SENSOR3_DISCONNECTED = 12
SENSOR3_BIAS = 13
SENSOR3_STUCK = 14
SENSOR3_DISCONNECTED_SENSOR1_BIAS = 15
THREE_WAY_DISAGREEMENT = 16
ALL_SENSORS_DISCONNECTED = 17
SENSOR1_OUT_OF_RANGE = 18
SENSOR2_OUT_OF_RANGE = 19
SENSOR3_OUT_OF_RANGE = 20
SENSOR1_STALE_ACQUISITION = 21
SENSOR2_STALE_ACQUISITION = 22
SENSOR3_STALE_ACQUISITION = 23

POLL_INTERVAL_SECONDS = 0.35
STATE_TIMEOUT_SECONDS = 35.0
TRANSPORT_RETRIES = 8

_CLIENT = None


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


def read_fault_mode():
    def operation(client):
        return require_response(
            client.read_holding_registers(
                HR_FAULT_MODE,
                count=1,
                device_id=DEVICE_ID,
            ),
            "read fault mode",
        ).registers[0]

    return with_transport_retry(operation)


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
            print(f"PASS {description}: {last}")
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
    return wait_for("recovery to healthy three-channel state", all_healthy)


def test_normal_and_heartbeat():
    normal = restore_normal()
    first_heartbeat = normal["heartbeat"]

    wait_for(
        "heartbeat changes on a later acquisition cycle",
        lambda s: s["heartbeat"] != first_heartbeat,
    )


def test_sensor1_disconnect_and_recovery():
    write_fault_mode(SENSOR1_DISCONNECTED)
    wait_for(
        "sensor 1 read failure",
        lambda s: (
            s["sensor1_health"] == CHANNEL_READ_FAILURE
            and s["temp1"] == INVALID
            and s["sensor2_health"] == CHANNEL_HEALTHY
            and s["sensor3_health"] == CHANNEL_HEALTHY
        ),
    )
    restore_normal()


def test_sensor3_out_of_range_and_recovery():
    write_fault_mode(SENSOR3_OUT_OF_RANGE)
    wait_for(
        "sensor 3 out-of-range health",
        lambda s: (
            s["sensor3_health"] == CHANNEL_OUT_OF_RANGE
            and s["temp3"] == INVALID
        ),
    )
    restore_normal()


def test_sensor2_stale_and_recovery():
    write_fault_mode(SENSOR2_STALE_ACQUISITION)
    wait_for(
        "sensor 2 stale acquisition timeout",
        lambda s: (
            s["sensor2_health"] == CHANNEL_STALE
            and s["temp2"] == INVALID
        ),
    )
    restore_normal()


def test_bias_remains_acquisition_healthy():
    restore_normal()
    write_fault_mode(SENSOR1_BIAS)

    snapshot = wait_for(
        "sensor 1 bias remains acquisition-healthy",
        lambda s: all_healthy(s) and abs(s["temp1"] - s["temp2"]) >= 150,
    )

    # This is intentional: disagreement/outlier interpretation belongs to PLC.
    assert snapshot["sensor1_health"] == CHANNEL_HEALTHY
    restore_normal()


def test_sensor1_stuck_and_recovery():
    restore_normal()
    write_fault_mode(SENSOR1_STUCK)

    wait_for(
        "sensor 1 stuck diagnostic",
        lambda s: (
            s["sensor1_health"] == CHANNEL_STUCK
            and s["temp1"] == INVALID
        ),
    )

    restore_normal()


def test_all_sensors_disconnected():
    write_fault_mode(ALL_SENSORS_DISCONNECTED)
    wait_for(
        "all sensors read-failed",
        lambda s: (
            s["sensor1_health"] == CHANNEL_READ_FAILURE
            and s["sensor2_health"] == CHANNEL_READ_FAILURE
            and s["sensor3_health"] == CHANNEL_READ_FAILURE
            and s["temp1"] == INVALID
            and s["temp2"] == INVALID
            and s["temp3"] == INVALID
        ),
    )
    restore_normal()


def test_invalid_fault_selector_fails_to_none():
    write_fault_mode(999)
    deadline = time.time() + STATE_TIMEOUT_SECONDS

    while time.time() < deadline:
        if read_fault_mode() == NONE:
            print("PASS invalid fault selector restored to NONE")
            restore_normal()
            return
        time.sleep(POLL_INTERVAL_SECONDS)

    raise AssertionError("Invalid fault selector was not restored to NONE")


def main():
    print("Virtual Modbus Interface v2 acquisition/diagnostic test campaign")
    print("Operational contract: 7 input registers; test selector: HR0")

    try:
        test_normal_and_heartbeat()
        test_sensor1_disconnect_and_recovery()
        test_sensor3_out_of_range_and_recovery()
        test_sensor2_stale_and_recovery()
        test_bias_remains_acquisition_healthy()
        test_sensor1_stuck_and_recovery()
        test_all_sensors_disconnected()
        test_invalid_fault_selector_fails_to_none()
    finally:
        try:
            write_fault_mode(NONE)
        except Exception:
            pass
        close_transport()

    print("PASS Virtual Modbus Interface v2 ESP32-side test campaign")


if __name__ == "__main__":
    main()
