import time

from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException


HOST = "127.0.0.1"
PORT = 1502
DEVICE_ID = 1

HR_FAULT_MODE = 1

IR_TEMP1 = 0
IR_TEMP2 = 1
IR_TCONTROL = 2
IR_SENSOR_DIFF = 3
IR_DIAGNOSTIC_STATUS = 6

DI_SENSOR1_VALID = 1
DI_SENSOR2_VALID = 2
DI_TCONTROL_VALID = 4

NONE = 0
SENSOR1_DISCONNECTED = 1
TEMPORARY_DISAGREEMENT = 8

BOTH_SENSORS_VALID = 0
SENSOR_1_FAULT = 1
SENSOR_DISAGREEMENT_PENDING = 7

INVALID = 65535
WRAPPED_DEVICE_DISCONNECTED = 52836
POLL_INTERVAL_SECONDS = 0.75
REQUEST_SPACING_SECONDS = 0.10
STATE_TIMEOUT_SECONDS = 15.0
TRANSPORT_RETRIES = 3


def require_response(response, operation):
    if response.isError():
        raise AssertionError(f"{operation} failed: {response}")
    return response


def connect_client():
    client = ModbusTcpClient(HOST, port=PORT)
    if not client.connect():
        raise ConnectionError(
            f"Could not connect to Modbus TCP at {HOST}:{PORT}"
        )
    return client


def write_fault_mode(mode):
    last_error = None

    for _ in range(TRANSPORT_RETRIES):
        client = None
        try:
            client = connect_client()
            response = client.write_register(
                HR_FAULT_MODE,
                mode,
                device_id=DEVICE_ID,
            )
            require_response(response, f"write fault mode {mode}")
            return
        except (ConnectionError, ModbusException, OSError) as error:
            last_error = error
            time.sleep(POLL_INTERVAL_SECONDS)
        finally:
            if client is not None:
                client.close()

    raise ConnectionError(
        f"Unable to write fault mode {mode}"
    ) from last_error


def read_snapshot():
    # Wokwi's forwarded Modbus connection can retain an out-of-order reply
    # during rapid polling. A paced, short-lived connection per snapshot
    # keeps the register and discrete-input reads together without backlog.
    last_error = None

    for _ in range(TRANSPORT_RETRIES):
        client = None
        try:
            client = connect_client()
            registers = require_response(
                client.read_input_registers(
                    0,
                    count=9,
                    device_id=DEVICE_ID,
                ),
                "read input registers",
            ).registers

            time.sleep(REQUEST_SPACING_SECONDS)

            discrete = require_response(
                client.read_discrete_inputs(
                    0,
                    count=8,
                    device_id=DEVICE_ID,
                ),
                "read discrete inputs",
            ).bits
            break
        except (ConnectionError, ModbusException, OSError) as error:
            last_error = error
            time.sleep(POLL_INTERVAL_SECONDS)
        finally:
            if client is not None:
                client.close()
    else:
        raise ConnectionError(
            "Unable to read a complete Modbus snapshot"
        ) from last_error

    return {
        "temp1": registers[IR_TEMP1],
        "temp2": registers[IR_TEMP2],
        "t_control": registers[IR_TCONTROL],
        "sensor_diff": registers[IR_SENSOR_DIFF],
        "diagnostic": registers[IR_DIAGNOSTIC_STATUS],
        "sensor1_valid": discrete[DI_SENSOR1_VALID],
        "sensor2_valid": discrete[DI_SENSOR2_VALID],
        "t_control_valid": discrete[DI_TCONTROL_VALID],
    }


def wait_for_snapshot(description, predicate):
    deadline = time.monotonic() + STATE_TIMEOUT_SECONDS
    last_snapshot = None

    while time.monotonic() < deadline:
        last_snapshot = read_snapshot()
        if predicate(last_snapshot):
            print(f"PASS {description}: {last_snapshot}")
            return last_snapshot
        time.sleep(POLL_INTERVAL_SECONDS)

    raise AssertionError(
        f"Timed out waiting for {description}; last={last_snapshot}"
    )


def disconnected_state(snapshot):
    return (
        snapshot["temp1"] == INVALID
        and snapshot["temp1"] != WRAPPED_DEVICE_DISCONNECTED
        and snapshot["temp2"] != INVALID
        and snapshot["t_control"] == snapshot["temp2"]
        and snapshot["sensor_diff"] == INVALID
        and snapshot["diagnostic"] == SENSOR_1_FAULT
        and not snapshot["sensor1_valid"]
        and snapshot["sensor2_valid"]
        and snapshot["t_control_valid"]
    )


def recovered_state(snapshot):
    return (
        snapshot["temp1"] != INVALID
        and snapshot["temp2"] != INVALID
        and snapshot["t_control"] != INVALID
        and snapshot["sensor_diff"] != INVALID
        and snapshot["diagnostic"] == BOTH_SENSORS_VALID
        and snapshot["sensor1_valid"]
        and snapshot["sensor2_valid"]
        and snapshot["t_control_valid"]
    )


def run_disconnect_and_recovery():
    write_fault_mode(NONE)
    wait_for_snapshot("initial NONE state", recovered_state)

    write_fault_mode(SENSOR1_DISCONNECTED)
    wait_for_snapshot(
        "SENSOR1_DISCONNECTED state",
        disconnected_state,
    )

    write_fault_mode(NONE)
    wait_for_snapshot("recovery to NONE", recovered_state)


def run_temporary_disagreement_rearm():
    write_fault_mode(NONE)
    baseline = wait_for_snapshot(
        "temporary-disagreement baseline",
        recovered_state,
    )

    write_fault_mode(TEMPORARY_DISAGREEMENT)
    one_shot = wait_for_snapshot(
        "temporary disagreement one-shot",
        lambda snapshot: (
            snapshot["diagnostic"]
            == SENSOR_DISAGREEMENT_PENDING
            and snapshot["sensor_diff"] >= 500
        ),
    )

    resumed = wait_for_snapshot(
        "temporary disagreement does not repeat",
        recovered_state,
    )

    time.sleep(1.5)
    still_resumed = read_snapshot()
    assert recovered_state(still_resumed), (
        "Temporary disagreement repeated without re-arm: "
        f"{still_resumed}"
    )

    write_fault_mode(NONE)
    time.sleep(1.5)
    wait_for_snapshot(
        "temporary disagreement re-arm NONE",
        recovered_state,
    )
    write_fault_mode(TEMPORARY_DISAGREEMENT)
    rearmed = wait_for_snapshot(
        "temporary disagreement fires after re-arm",
        lambda snapshot: (
            snapshot["diagnostic"]
            == SENSOR_DISAGREEMENT_PENDING
            and snapshot["sensor_diff"] >= 500
        ),
    )

    print(
        "Temporary disagreement evidence:",
        {
            "baseline": baseline,
            "one_shot": one_shot,
            "resumed": resumed,
            "rearmed": rearmed,
        },
    )


def main():
    try:
        run_disconnect_and_recovery()
        run_temporary_disagreement_rearm()
        print("ALL FAULT-INJECTION TESTS PASSED")
    finally:
        write_fault_mode(NONE)


if __name__ == "__main__":
    main()
