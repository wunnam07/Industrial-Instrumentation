import time

from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException


HOST = "127.0.0.1"
PORT = 1502
DEVICE_ID = 1

HR_PHYSICAL_PLC_HEARTBEAT = 0
HR_FAULT_MODE = 1
COIL_PHYSICAL_HEATER_DEMAND = 0

IR_TEMP1 = 0
IR_TEMP2 = 1
IR_ESP32_TEMP_CANDIDATE = 2
IR_SENSOR_DIFF_12 = 3
IR_DIAGNOSTIC_STATUS = 6
IR_LEGACY_PROCESS_STATE = 7
IR_ESP32_HEARTBEAT = 8
IR_TEMP3 = 9
IR_SENSOR_DIFF_13 = 10
IR_SENSOR_DIFF_23 = 11
IR_VOTING_STATUS = 12
IR_SENSOR1_HEALTH = 13
IR_SENSOR2_HEALTH = 14
IR_SENSOR3_HEALTH = 15
IR_SENSOR1_RECOVERY = 16
IR_SENSOR2_RECOVERY = 17
IR_SENSOR3_RECOVERY = 18

DI_INSTRUMENT_NODE_READY = 0
DI_SENSOR1_VALID = 1
DI_SENSOR2_VALID = 2
DI_AGREE_12 = 3
DI_TEMP_CANDIDATE_VALID = 4
DI_PHYSICAL_OUTPUT_PERMISSIVE = 5
DI_PHYSICAL_HEATER_OUTPUT = 6
DI_PHYSICAL_PLC_COMMS_OK = 7
DI_SENSOR3_VALID = 8
DI_AGREE_13 = 9
DI_AGREE_23 = 10
DI_VOTING_RESOLVED = 11

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
SENSOR1_DISCONNECTED_SENSOR2_BIAS = 24
SENSOR1_DISCONNECTED_SENSOR2_STUCK = 25
SENSOR1_OUT_OF_RANGE_SENSOR3_BIAS = 26
SENSOR1_SENSOR2_CORRELATED_BIAS = 27
SENSOR2_SENSOR3_CORRELATED_BIAS = 28
SENSOR1_SENSOR2_CORRELATED_STUCK = 29
SENSOR1_SENSOR2_CORRELATED_DRIFT = 30
SENSOR1_HIGH_SENSOR2_LOW = 31
SENSOR1_SENSOR3_DISCONNECTED = 32
SENSOR2_SENSOR3_DISCONNECTED = 33

ALL_THREE_SENSORS_VALID = 0
SENSOR_1_FAULT = 1
SENSOR_2_FAULT = 2
SENSOR_DISAGREEMENT = 3
NO_VALID_SENSOR = 4
SENSOR_1_STUCK = 5
SENSOR_2_STUCK = 6
SENSOR_DISAGREEMENT_PENDING = 7
SENSOR_3_FAULT = 8
SENSOR_1_OUTLIER = 9
SENSOR_2_OUTLIER = 10
SENSOR_3_OUTLIER = 11
SENSOR_3_STUCK = 12
INSUFFICIENT_REDUNDANCY = 13

VOTE_UNRESOLVED = 0
VOTE_ALL_THREE = 1
VOTE_PAIR_12 = 2
VOTE_PAIR_13 = 3
VOTE_PAIR_23 = 4
VOTE_SINGLE_1 = 5
VOTE_SINGLE_2 = 6
VOTE_SINGLE_3 = 7

CHANNEL_HEALTHY = 0
CHANNEL_READ_FAILURE = 1
CHANNEL_OUT_OF_RANGE = 2
CHANNEL_STALE = 3
CHANNEL_RECOVERING = 4
CHANNEL_RECOVERY_SAMPLES = 3

INVALID = 65535
BIAS_OFFSET = 200
DISAGREEMENT_OFFSET = 500
RAMP_STEP = 25
RAMP_CAP = 200

POLL_INTERVAL_SECONDS = 0.35
REQUEST_SPACING_SECONDS = 0.08
STATE_TIMEOUT_SECONDS = 60.0
LONG_STATE_TIMEOUT_SECONDS = 180.0
TRANSPORT_RETRIES = 8

RESULTS = []
ROBUSTNESS_RESULTS = []
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


def write_physical_plc_heartbeat(value):
    def operation(client):
        return require_response(
            client.write_register(
                HR_PHYSICAL_PLC_HEARTBEAT,
                value,
                device_id=DEVICE_ID,
            ),
            f"write PLC heartbeat {value}",
        )

    with_transport_retry(operation)


def write_physical_heater_demand(enabled):
    def operation(client):
        return require_response(
            client.write_coil(
                COIL_PHYSICAL_HEATER_DEMAND,
                enabled,
                device_id=DEVICE_ID,
            ),
            f"write heater command {enabled}",
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
                count=19,
                device_id=DEVICE_ID,
            ),
            "read input registers",
        ).registers

        time.sleep(REQUEST_SPACING_SECONDS)

        discrete = require_response(
            client.read_discrete_inputs(
                0,
                count=12,
                device_id=DEVICE_ID,
            ),
            "read discrete inputs",
        ).bits

        return {
            "temp1": registers[IR_TEMP1],
            "temp2": registers[IR_TEMP2],
            "temp3": registers[IR_TEMP3],
            "temp_candidate": registers[IR_ESP32_TEMP_CANDIDATE],
            "diff12": registers[IR_SENSOR_DIFF_12],
            "diff13": registers[IR_SENSOR_DIFF_13],
            "diff23": registers[IR_SENSOR_DIFF_23],
            "diagnostic": registers[IR_DIAGNOSTIC_STATUS],
            "legacy_process_state": registers[IR_LEGACY_PROCESS_STATE],
            "heartbeat": registers[IR_ESP32_HEARTBEAT],
            "vote": registers[IR_VOTING_STATUS],
            "sensor1_health": registers[IR_SENSOR1_HEALTH],
            "sensor2_health": registers[IR_SENSOR2_HEALTH],
            "sensor3_health": registers[IR_SENSOR3_HEALTH],
            "sensor1_recovery": registers[IR_SENSOR1_RECOVERY],
            "sensor2_recovery": registers[IR_SENSOR2_RECOVERY],
            "sensor3_recovery": registers[IR_SENSOR3_RECOVERY],
            "node_ready": discrete[DI_INSTRUMENT_NODE_READY],
            "sensor1_valid": discrete[DI_SENSOR1_VALID],
            "sensor2_valid": discrete[DI_SENSOR2_VALID],
            "sensor3_valid": discrete[DI_SENSOR3_VALID],
            "agree12": discrete[DI_AGREE_12],
            "agree13": discrete[DI_AGREE_13],
            "agree23": discrete[DI_AGREE_23],
            "temp_candidate_valid": discrete[DI_TEMP_CANDIDATE_VALID],
            "physical_output_permissive":
                discrete[DI_PHYSICAL_OUTPUT_PERMISSIVE],
            "physical_heater_output":
                discrete[DI_PHYSICAL_HEATER_OUTPUT],
            "physical_plc_comms_ok":
                discrete[DI_PHYSICAL_PLC_COMMS_OK],
            "voting_resolved": discrete[DI_VOTING_RESOLVED],
        }

    return with_transport_retry(operation)


def wait_for_snapshot(description, predicate, timeout=STATE_TIMEOUT_SECONDS):
    deadline = time.monotonic() + timeout
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


def next_sample(previous_heartbeat, description):
    deadline = time.monotonic() + STATE_TIMEOUT_SECONDS
    last_snapshot = None
    while time.monotonic() < deadline:
        last_snapshot = read_snapshot()
        delta = (
            last_snapshot["heartbeat"] - previous_heartbeat
        ) & 0xFFFF
        if delta > 0:
            return last_snapshot, delta
        time.sleep(POLL_INTERVAL_SECONDS)
    raise AssertionError(
        f"Timed out waiting for {description}; last={last_snapshot}"
    )


def healthy(snapshot):
    return (
        snapshot["node_ready"]
        and snapshot["sensor1_valid"]
        and snapshot["sensor2_valid"]
        and snapshot["sensor3_valid"]
        and snapshot["agree12"]
        and snapshot["agree13"]
        and snapshot["agree23"]
        and snapshot["diagnostic"] == ALL_THREE_SENSORS_VALID
        and snapshot["vote"] == VOTE_ALL_THREE
        and snapshot["voting_resolved"]
        and snapshot["temp_candidate_valid"]
        and snapshot["temp1"] != INVALID
        and snapshot["temp2"] != INVALID
        and snapshot["temp3"] != INVALID
        and snapshot["temp_candidate"] != INVALID
        and snapshot["legacy_process_state"] == INVALID
        and not snapshot["physical_output_permissive"]
        and not snapshot["physical_heater_output"]
        and not snapshot["physical_plc_comms_ok"]
        and all(
            snapshot[f"sensor{sensor}_health"] == CHANNEL_HEALTHY
            and snapshot[f"sensor{sensor}_recovery"]
            == CHANNEL_RECOVERY_SAMPLES
            for sensor in (1, 2, 3)
        )
    )


def restore_none(description="recovery to NONE"):
    previous_heartbeat = read_snapshot()["heartbeat"]
    write_fault_mode(NONE)
    return wait_for_snapshot(
        description,
        lambda value: (
            healthy(value)
            and (
                (value["heartbeat"] - previous_heartbeat) & 0xFFFF
            ) > 0
        ),
        timeout=LONG_STATE_TIMEOUT_SECONDS,
    )


def record(name, evidence):
    RESULTS.append((name, True, evidence))
    print(f"PASS {name}: {evidence}")


def record_robustness(
    name,
    evidence,
    detection,
    isolation,
    accommodation,
    safety,
    architectural_limitation,
):
    classification = {
        "detection": detection,
        "isolation": isolation,
        "accommodation": accommodation,
        "safety": safety,
        "architectural_limitation": architectural_limitation,
    }
    ROBUSTNESS_RESULTS.append((name, classification))
    record(
        name,
        {
            "classification": classification,
            "evidence": evidence,
        },
    )


def assert_instrumentation_invariants(snapshot):
    assert snapshot["legacy_process_state"] == INVALID, snapshot
    assert not snapshot["physical_output_permissive"], snapshot
    assert not snapshot["physical_heater_output"], snapshot
    assert not snapshot["physical_plc_comms_ok"], snapshot

    for sensor_number in (1, 2, 3):
        valid = snapshot[f"sensor{sensor_number}_valid"]
        health = snapshot[f"sensor{sensor_number}_health"]
        if not valid:
            assert snapshot[f"temp{sensor_number}"] == INVALID, snapshot
        if health == CHANNEL_RECOVERING:
            assert not valid, snapshot

    if snapshot["diagnostic"] == SENSOR_DISAGREEMENT:
        assert snapshot["temp_candidate"] == INVALID, snapshot
        assert not snapshot["temp_candidate_valid"], snapshot


def run_case(name, expected, operation):
    try:
        operation()
    except Exception as error:
        RESULTS.append((name, False, str(error)))
        print(f"FAIL {name}")
        print(f"Expected: {expected}")
        print(f"Observed: {error}")
        raise


def assert_channel_state(snapshot, sensor_number, health, usable):
    assert snapshot[f"sensor{sensor_number}_health"] == health, snapshot
    assert snapshot[f"sensor{sensor_number}_valid"] is usable, snapshot
    if usable:
        assert snapshot[f"temp{sensor_number}"] != INVALID, snapshot
    else:
        assert snapshot[f"temp{sensor_number}"] == INVALID, snapshot


def recover_channel(sensor_number, description):
    write_fault_mode(NONE)
    first = wait_for_snapshot(
        f"{description} recovery sample 1",
        lambda value: (
            value[f"sensor{sensor_number}_health"] == CHANNEL_RECOVERING
            and value[f"sensor{sensor_number}_recovery"] == 1
            and not value[f"sensor{sensor_number}_valid"]
            and value[f"temp{sensor_number}"] == INVALID
        ),
        timeout=LONG_STATE_TIMEOUT_SECONDS,
    )

    second, _ = next_sample(
        first["heartbeat"],
        f"{description} recovery sample 2",
    )
    assert_channel_state(
        second,
        sensor_number,
        CHANNEL_RECOVERING,
        False,
    )
    assert second[f"sensor{sensor_number}_recovery"] == 2, second

    rejoined, _ = next_sample(
        second["heartbeat"],
        f"{description} qualified re-entry",
    )
    assert_channel_state(
        rejoined,
        sensor_number,
        CHANNEL_HEALTHY,
        True,
    )
    assert (
        rejoined[f"sensor{sensor_number}_recovery"]
        == CHANNEL_RECOVERY_SAMPLES
    ), rejoined
    assert healthy(rejoined), rejoined
    print(f"PASS {description} qualified re-entry: {rejoined}")
    return {"first": first, "second": second, "rejoined": rejoined}


def test_healthy_operation():
    snapshot = restore_none("healthy three-sensor baseline")
    assert read_fault_mode() == NONE
    assert snapshot["temp_candidate"] == round(
        (snapshot["temp1"] + snapshot["temp2"] + snapshot["temp3"])
        / 3
    )
    record("healthy three-sensor operation", snapshot)


def test_outlier(mode, sensor_number, diagnostic, vote):
    baseline = restore_none(f"Sensor {sensor_number} outlier baseline")
    write_fault_mode(mode)

    snapshot = wait_for_snapshot(
        f"Sensor {sensor_number} outlier isolation",
        lambda value: (
            value["diagnostic"] == diagnostic
            and value["vote"] == vote
            and value["temp_candidate_valid"]
        ),
    )

    temperatures = [
        snapshot["temp1"],
        snapshot["temp2"],
        snapshot["temp3"],
    ]
    baseline_temperatures = [
        baseline["temp1"],
        baseline["temp2"],
        baseline["temp3"],
    ]
    selected_index = sensor_number - 1
    assert temperatures[selected_index] == (
        baseline_temperatures[selected_index] + BIAS_OFFSET
    )
    peers = [
        temperature
        for index, temperature in enumerate(temperatures)
        if index != selected_index
    ]
    assert snapshot["temp_candidate"] == round(sum(peers) / 2)
    assert all(
        snapshot[key]
        for key in (
            "sensor1_valid",
            "sensor2_valid",
            "sensor3_valid",
        )
    )
    assert all(
        snapshot[f"sensor{sensor}_health"] == CHANNEL_HEALTHY
        for sensor in (1, 2, 3)
    )

    record(f"Sensor {sensor_number} outlier", snapshot)
    restore_none(f"Sensor {sensor_number} outlier recovery")


def test_disconnect(mode, sensor_number, diagnostic, vote):
    restore_none(f"Sensor {sensor_number} disconnect baseline")
    write_fault_mode(mode)
    snapshot = wait_for_snapshot(
        f"Sensor {sensor_number} disconnected",
        lambda value: (
            value[f"temp{sensor_number}"] == INVALID
            and not value[f"sensor{sensor_number}_valid"]
            and value["diagnostic"] == diagnostic
            and value["vote"] == vote
            and value["temp_candidate_valid"]
        ),
    )

    peers = [
        snapshot[f"temp{index}"]
        for index in (1, 2, 3)
        if index != sensor_number
    ]
    assert snapshot["temp_candidate"] == round(sum(peers) / 2)
    assert all(value != INVALID for value in peers)

    pair_names = {
        1: ("diff12", "diff13"),
        2: ("diff12", "diff23"),
        3: ("diff13", "diff23"),
    }
    assert all(snapshot[name] == INVALID for name in pair_names[sensor_number])
    assert_channel_state(
        snapshot,
        sensor_number,
        CHANNEL_READ_FAILURE,
        False,
    )
    assert snapshot[f"sensor{sensor_number}_recovery"] == 0

    recovery = recover_channel(
        sensor_number,
        f"Sensor {sensor_number} disconnect",
    )
    record(
        f"Sensor {sensor_number} disconnect and qualified recovery",
        {"fault": snapshot, "recovery": recovery},
    )


def test_out_of_range(mode, sensor_number, diagnostic, vote):
    restore_none(f"Sensor {sensor_number} out-of-range baseline")
    write_fault_mode(mode)
    snapshot = wait_for_snapshot(
        f"Sensor {sensor_number} out of range",
        lambda value: (
            value[f"sensor{sensor_number}_health"]
            == CHANNEL_OUT_OF_RANGE
            and not value[f"sensor{sensor_number}_valid"]
            and value[f"temp{sensor_number}"] == INVALID
            and value["diagnostic"] == diagnostic
            and value["vote"] == vote
            and value["temp_candidate_valid"]
        ),
    )
    assert snapshot[f"sensor{sensor_number}_recovery"] == 0
    peers = [
        snapshot[f"temp{index}"]
        for index in (1, 2, 3)
        if index != sensor_number
    ]
    assert snapshot["temp_candidate"] == round(sum(peers) / 2)

    recovery = recover_channel(
        sensor_number,
        f"Sensor {sensor_number} out-of-range",
    )
    record(
        f"Sensor {sensor_number} out-of-range and qualified recovery",
        {"fault": snapshot, "recovery": recovery},
    )


def test_constant_fresh_measurements():
    baseline = restore_none("constant fresh baseline")
    heartbeat = baseline["heartbeat"]
    observations = []

    for sample_number in range(4):
        snapshot, _ = next_sample(
            heartbeat,
            f"constant fresh sample {sample_number + 1}",
        )
        heartbeat = snapshot["heartbeat"]
        observations.append(snapshot)
        assert healthy(snapshot), snapshot
        assert (
            snapshot["temp1"],
            snapshot["temp2"],
            snapshot["temp3"],
        ) == (
            baseline["temp1"],
            baseline["temp2"],
            baseline["temp3"],
        )

    record(
        "constant fresh measurements are not stale",
        {"samples": len(observations), "final": observations[-1]},
    )


def test_stale_acquisition(mode, sensor_number, diagnostic, vote):
    baseline = restore_none(f"Sensor {sensor_number} stale baseline")
    write_fault_mode(mode)

    first_missed, _ = next_sample(
        baseline["heartbeat"],
        f"Sensor {sensor_number} first suppressed acquisition",
    )
    assert_channel_state(
        first_missed,
        sensor_number,
        CHANNEL_HEALTHY,
        True,
    )
    assert first_missed[f"temp{sensor_number}"] == baseline[
        f"temp{sensor_number}"
    ]

    stale = wait_for_snapshot(
        f"Sensor {sensor_number} stale exclusion",
        lambda value: (
            value[f"sensor{sensor_number}_health"] == CHANNEL_STALE
            and value[f"sensor{sensor_number}_recovery"] == 0
            and not value[f"sensor{sensor_number}_valid"]
            and value[f"temp{sensor_number}"] == INVALID
            and value["diagnostic"] == diagnostic
            and value["vote"] == vote
            and value["temp_candidate_valid"]
        ),
        timeout=LONG_STATE_TIMEOUT_SECONDS,
    )

    recovery = recover_channel(
        sensor_number,
        f"Sensor {sensor_number} stale",
    )
    record(
        f"Sensor {sensor_number} stale exclusion and recovery",
        {
            "first_missed": first_missed,
            "stale": stale,
            "recovery": recovery,
        },
    )


def test_recovery_interruption():
    restore_none("recovery interruption baseline")
    write_fault_mode(SENSOR1_DISCONNECTED)
    initial_fault = wait_for_snapshot(
        "recovery interruption initial fault",
        lambda value: (
            value["sensor1_health"] == CHANNEL_READ_FAILURE
            and value["sensor1_recovery"] == 0
        ),
    )

    write_fault_mode(NONE)
    recovering = wait_for_snapshot(
        "recovery interruption first healthy sample",
        lambda value: (
            value["sensor1_health"] == CHANNEL_RECOVERING
            and value["sensor1_recovery"] == 1
            and not value["sensor1_valid"]
        ),
    )

    write_fault_mode(SENSOR1_DISCONNECTED)
    interrupted = wait_for_snapshot(
        "recovery interruption resets qualification",
        lambda value: (
            value["sensor1_health"] == CHANNEL_READ_FAILURE
            and value["sensor1_recovery"] == 0
            and not value["sensor1_valid"]
        ),
    )
    assert interrupted["heartbeat"] != recovering["heartbeat"]

    recovery = recover_channel(1, "recovery interruption")
    record(
        "recovery qualification resets on renewed fault",
        {
            "initial_fault": initial_fault,
            "recovering": recovering,
            "interrupted": interrupted,
            "recovery": recovery,
        },
    )


def test_degraded_invalid_counts():
    restore_none("two-invalid baseline")
    write_fault_mode(BOTH_SENSORS_DISCONNECTED)
    one_remaining = wait_for_snapshot(
        "two explicitly invalid sensors",
        lambda value: (
            not value["sensor1_valid"]
            and not value["sensor2_valid"]
            and value["sensor3_valid"]
            and value["vote"] == VOTE_SINGLE_3
            and value["diagnostic"] == INSUFFICIENT_REDUNDANCY
            and value["temp_candidate"] == value["temp3"]
            and value["temp_candidate_valid"]
        ),
    )
    assert one_remaining["sensor1_health"] == CHANNEL_READ_FAILURE
    assert one_remaining["sensor2_health"] == CHANNEL_READ_FAILURE
    assert one_remaining["sensor3_health"] == CHANNEL_HEALTHY
    record("single-sensor degraded operation", one_remaining)

    restore_none("all-invalid baseline")
    write_fault_mode(ALL_SENSORS_DISCONNECTED)
    none_remaining = wait_for_snapshot(
        "all three sensors invalid",
        lambda value: (
            not value["sensor1_valid"]
            and not value["sensor2_valid"]
            and not value["sensor3_valid"]
            and value["diagnostic"] == NO_VALID_SENSOR
            and value["temp_candidate"] == INVALID
            and not value["temp_candidate_valid"]
        ),
    )
    assert all(
        none_remaining[f"sensor{sensor}_health"]
        == CHANNEL_READ_FAILURE
        for sensor in (1, 2, 3)
    )
    record("all sensors invalid", none_remaining)
    restore_none("all-invalid recovery")


def test_temporary_outlier_rearm():
    restore_none("temporary outlier baseline")
    write_fault_mode(TEMPORARY_DISAGREEMENT)
    first = wait_for_snapshot(
        "temporary outlier one-shot",
        lambda value: (
            value["diagnostic"] == SENSOR_1_OUTLIER
            and value["diff12"] == DISAGREEMENT_OFFSET
            and value["vote"] == VOTE_PAIR_23
        ),
    )
    resumed = wait_for_snapshot(
        "temporary outlier does not repeat",
        healthy,
    )
    time.sleep(1.5)
    assert healthy(read_snapshot())

    none_sample = restore_none("temporary outlier re-arm")
    none_observed, _ = next_sample(
        none_sample["heartbeat"],
        "temporary outlier NONE mode observation",
    )
    assert healthy(none_observed), none_observed
    write_fault_mode(TEMPORARY_DISAGREEMENT)
    rearmed = wait_for_snapshot(
        "temporary outlier fires after re-arm",
        lambda value: value["diagnostic"] == SENSOR_1_OUTLIER,
    )
    record(
        "temporary outlier one-shot and re-arm",
        {"first": first, "resumed": resumed, "rearmed": rearmed},
    )
    restore_none("temporary outlier recovery")


def test_persistent_outlier():
    baseline = restore_none("persistent outlier baseline")
    write_fault_mode(PERSISTENT_DISAGREEMENT)
    first = wait_for_snapshot(
        "persistent Sensor 1 outlier",
        lambda value: (
            value["diagnostic"] == SENSOR_1_OUTLIER
            and value["vote"] == VOTE_PAIR_23
            and value["diff12"] == DISAGREEMENT_OFFSET
            and value["temp_candidate"] == baseline["temp2"]
        ),
    )
    second, _ = next_sample(
        first["heartbeat"],
        "persistent outlier held sample",
    )
    assert second["diagnostic"] == SENSOR_1_OUTLIER, second
    assert second["diff12"] == DISAGREEMENT_OFFSET, second
    record(
        "persistent disagreement isolated by majority",
        {"first": first, "held": second},
    )
    restore_none("persistent outlier recovery")


def test_unresolved(mode, name, third_sensor_invalid):
    baseline = restore_none(f"{name} baseline")
    write_fault_mode(mode)

    pending = wait_for_snapshot(
        f"{name} pending",
        lambda value: (
            value["diagnostic"] == SENSOR_DISAGREEMENT_PENDING
            and value["temp_candidate_valid"]
            and value["temp_candidate"] == baseline["temp_candidate"]
            and value["vote"] == VOTE_UNRESOLVED
        ),
    )
    confirmed = wait_for_snapshot(
        f"{name} confirmed",
        lambda value: (
            value["diagnostic"] == SENSOR_DISAGREEMENT
            and value["temp_candidate"] == INVALID
            and not value["temp_candidate_valid"]
        ),
    )

    if third_sensor_invalid:
        assert not confirmed["sensor3_valid"]
        assert confirmed["temp3"] == INVALID
        assert confirmed["diff12"] == DISAGREEMENT_OFFSET
    else:
        assert all(
            confirmed[key]
            for key in (
                "sensor1_valid",
                "sensor2_valid",
                "sensor3_valid",
            )
        )
        assert not confirmed["agree12"]
        assert not confirmed["agree13"]
        assert not confirmed["agree23"]

    record(name, {"pending": pending, "confirmed": confirmed})
    restore_none(f"{name} recovery")


def test_stuck(
    mode,
    sensor_number,
    diagnostic,
    vote,
    phase_delay_samples=0,
):
    baseline = restore_none(
        f"Sensor {sensor_number} stuck baseline phase {phase_delay_samples}"
    )
    heartbeat = baseline["heartbeat"]
    for sample_number in range(phase_delay_samples):
        baseline, _ = next_sample(
            heartbeat,
            f"stuck phase delay {sample_number + 1}",
        )
        heartbeat = baseline["heartbeat"]

    write_fault_mode(mode)
    detected = wait_for_snapshot(
        f"Sensor {sensor_number} stuck detection phase {phase_delay_samples}",
        lambda value: (
            value["diagnostic"] == diagnostic
            and value["vote"] == vote
            and value["temp_candidate_valid"]
        ),
        timeout=LONG_STATE_TIMEOUT_SECONDS,
    )

    frozen_key = f"temp{sensor_number}"
    assert detected[frozen_key] == baseline[frozen_key]
    peers = [
        detected[f"temp{index}"]
        for index in (1, 2, 3)
        if index != sensor_number
    ]
    assert peers[0] == peers[1]
    assert peers[0] >= baseline["temp1"] + 50
    assert detected["temp_candidate"] == round(sum(peers) / 2)
    assert all(
        detected[f"sensor{sensor}_health"] == CHANNEL_HEALTHY
        and detected[f"sensor{sensor}_valid"]
        for sensor in (1, 2, 3)
    )

    record(
        f"Sensor {sensor_number} stuck phase {phase_delay_samples}",
        detected,
    )
    restore_none(
        f"Sensor {sensor_number} stuck recovery phase {phase_delay_samples}"
    )


def test_common_process_change():
    baseline = restore_none("common process baseline")
    heartbeat = baseline["heartbeat"]
    write_fault_mode(COMMON_PROCESS_CHANGE)
    observations = []

    while len(observations) < 10:
        snapshot, _ = next_sample(heartbeat, "common process sample")
        heartbeat = snapshot["heartbeat"]
        observations.append(snapshot)
        assert snapshot["diagnostic"] == ALL_THREE_SENSORS_VALID
        assert snapshot["temp1"] == snapshot["temp2"] == snapshot["temp3"]
        assert snapshot["temp_candidate"] == snapshot["temp1"]
        assert snapshot["temp_candidate_valid"]

    assert observations[-1]["temp1"] == baseline["temp1"] + RAMP_CAP
    record(
        "common process change without false stuck",
        {"samples": len(observations), "final": observations[-1]},
    )
    restore_none("common process recovery")


def test_drift_vs_stuck():
    baseline = restore_none("drift ambiguity baseline")
    heartbeat = baseline["heartbeat"]
    write_fault_mode(SENSOR1_DRIFT)
    observations = []

    while len(observations) < 10:
        snapshot, _ = next_sample(heartbeat, "drift sample")
        heartbeat = snapshot["heartbeat"]
        observations.append(snapshot)
        assert snapshot["temp2"] == baseline["temp2"]
        assert snapshot["temp3"] == baseline["temp3"]
        assert snapshot["diagnostic"] not in (
            SENSOR_1_STUCK,
            SENSOR_2_STUCK,
            SENSOR_3_STUCK,
        )
        assert all(
            snapshot[f"sensor{sensor}_health"] == CHANNEL_HEALTHY
            for sensor in (1, 2, 3)
        )

    isolated = [
        sample
        for sample in observations
        if sample["diagnostic"] == SENSOR_1_OUTLIER
    ]
    assert isolated, observations
    assert isolated[-1]["vote"] == VOTE_PAIR_23
    assert isolated[-1]["temp_candidate"] == baseline["temp2"]
    record(
        "drift versus stuck ambiguity",
        {"samples": len(observations), "final": observations[-1]},
    )
    restore_none("drift ambiguity recovery")


def test_invalid_mode():
    restore_none("invalid mode baseline")
    write_fault_mode(999)
    deadline = time.monotonic() + STATE_TIMEOUT_SECONDS
    while time.monotonic() < deadline and read_fault_mode() != NONE:
        time.sleep(POLL_INTERVAL_SECONDS)
    assert read_fault_mode() == NONE
    snapshot = wait_for_snapshot("invalid mode normal acquisition", healthy)
    record("invalid mode fail-safe", snapshot)


def test_simulated_responsibility_boundary():
    baseline = restore_none("simulated responsibility baseline")
    start_heartbeat = baseline["heartbeat"]

    # Legacy physical-output command points remain address-compatible but are
    # deliberately ignored by the active simulated instrument-node build.
    write_physical_heater_demand(True)
    write_physical_plc_heartbeat(
        (int(time.monotonic()) + 1) & 0xFFFF
    )
    ignored = wait_for_snapshot(
        "physical command ignored by simulated build",
        lambda value: (
            ((value["heartbeat"] - start_heartbeat) & 0xFFFF) > 0
            and value["node_ready"]
            and value["legacy_process_state"] == INVALID
            and not value["physical_output_permissive"]
            and not value["physical_heater_output"]
            and not value["physical_plc_comms_ok"]
            and healthy(value)
        ),
        timeout=LONG_STATE_TIMEOUT_SECONDS,
    )
    write_physical_heater_demand(False)
    record(
        "simulated ESP32 has no process-control authority",
        ignored,
    )


def capture_confirmed_unresolved(mode, description):
    baseline = restore_none(f"{description} baseline")
    write_fault_mode(mode)
    pending = wait_for_snapshot(
        f"{description} pending",
        lambda value: (
            value["vote"] == VOTE_UNRESOLVED
            and value["diagnostic"] == SENSOR_DISAGREEMENT_PENDING
            and value["temp_candidate_valid"]
            and value["temp_candidate"] == baseline["temp_candidate"]
        ),
        timeout=LONG_STATE_TIMEOUT_SECONDS,
    )
    confirmed = wait_for_snapshot(
        f"{description} confirmed",
        lambda value: (
            value["vote"] == VOTE_UNRESOLVED
            and value["diagnostic"] == SENSOR_DISAGREEMENT
            and value["temp_candidate"] == INVALID
            and not value["temp_candidate_valid"]
        ),
        timeout=LONG_STATE_TIMEOUT_SECONDS,
    )
    assert_instrumentation_invariants(pending)
    assert_instrumentation_invariants(confirmed)
    return baseline, pending, confirmed


def test_robustness_channel_failure_plus_outlier():
    scenarios = (
        (
            "A1 S1 disconnected plus S2 high",
            SENSOR1_DISCONNECTED_SENSOR2_BIAS,
            1,
            2,
            3,
        ),
        (
            "A2 S3 disconnected plus S1 high",
            SENSOR3_DISCONNECTED_SENSOR1_BIAS,
            3,
            1,
            2,
        ),
    )

    for name, mode, failed, biased, healthy_sensor in scenarios:
        baseline, pending, confirmed = capture_confirmed_unresolved(
            mode,
            name,
        )
        assert confirmed[f"sensor{failed}_health"] == CHANNEL_READ_FAILURE
        assert not confirmed[f"sensor{failed}_valid"]
        assert confirmed[f"sensor{biased}_health"] == CHANNEL_HEALTHY
        assert confirmed[f"sensor{healthy_sensor}_health"] == CHANNEL_HEALTHY
        assert confirmed[f"temp{biased}"] == baseline[f"temp{biased}"] + 500
        record_robustness(
            name,
            {"pending": pending, "confirmed": confirmed},
            "yes",
            "ambiguous between the two usable sensors",
            "last diagnostic candidate while pending; none after confirmation",
            "safe",
            "yes",
        )
        restore_none(f"{name} recovery")


def test_robustness_channel_failure_plus_stuck():
    baseline = restore_none("B channel failure plus stuck baseline")
    heartbeat = baseline["heartbeat"]
    write_fault_mode(SENSOR1_DISCONNECTED_SENSOR2_STUCK)
    observations = []
    pending_seen = False
    confirmed = None

    while len(observations) < 15:
        snapshot, _ = next_sample(
            heartbeat,
            "B channel failure plus stuck sample",
        )
        heartbeat = snapshot["heartbeat"]
        observations.append(snapshot)
        assert snapshot["diagnostic"] not in (
            SENSOR_1_STUCK,
            SENSOR_2_STUCK,
            SENSOR_3_STUCK,
        ), snapshot
        if snapshot["diagnostic"] == SENSOR_DISAGREEMENT_PENDING:
            pending_seen = True
        if snapshot["diagnostic"] == SENSOR_DISAGREEMENT:
            confirmed = snapshot
            break

    assert pending_seen, observations
    assert confirmed is not None, observations
    assert confirmed["sensor1_health"] == CHANNEL_READ_FAILURE
    assert confirmed["sensor2_health"] == CHANNEL_HEALTHY
    assert confirmed["sensor3_health"] == CHANNEL_HEALTHY
    assert confirmed["temp2"] == baseline["temp2"]
    assert confirmed["temp3"] > baseline["temp3"] + 100
    assert_instrumentation_invariants(confirmed)
    record_robustness(
        "B disconnected channel plus stuck measurement",
        {"samples": len(observations), "confirmed": confirmed},
        "yes",
        "ambiguous; no false stuck isolation without a third voter",
        "none after disagreement confirmation",
        "safe",
        "yes",
    )
    restore_none("B channel failure plus stuck recovery")


def test_robustness_out_of_range_plus_disagreement():
    baseline, pending, confirmed = capture_confirmed_unresolved(
        SENSOR1_OUT_OF_RANGE_SENSOR3_BIAS,
        "C S1 out of range plus S2/S3 disagreement",
    )
    assert confirmed["sensor1_health"] == CHANNEL_OUT_OF_RANGE
    assert not confirmed["sensor1_valid"]
    assert confirmed["sensor2_health"] == CHANNEL_HEALTHY
    assert confirmed["sensor3_health"] == CHANNEL_HEALTHY
    assert confirmed["temp3"] == baseline["temp3"] + 500
    record_robustness(
        "C out-of-range channel plus remaining disagreement",
        {"pending": pending, "confirmed": confirmed},
        "yes",
        "ambiguous between Sensors 2 and 3",
        "last diagnostic candidate while pending; none after confirmation",
        "safe",
        "yes",
    )
    restore_none("C out-of-range plus disagreement recovery")


def test_robustness_two_explicit_channel_failures():
    scenarios = (
        (
            "D1 S1/S2 disconnected",
            BOTH_SENSORS_DISCONNECTED,
            (1, 2),
            3,
            VOTE_SINGLE_3,
        ),
        (
            "D2 S1/S3 disconnected",
            SENSOR1_SENSOR3_DISCONNECTED,
            (1, 3),
            2,
            VOTE_SINGLE_2,
        ),
        (
            "D3 S2/S3 disconnected",
            SENSOR2_SENSOR3_DISCONNECTED,
            (2, 3),
            1,
            VOTE_SINGLE_1,
        ),
    )

    for name, mode, failed_sensors, remaining, vote in scenarios:
        baseline = restore_none(f"{name} baseline")
        write_fault_mode(mode)
        degraded = wait_for_snapshot(
            f"{name} degraded single-sensor evidence",
            lambda value: (
                value["vote"] == vote
                and value["diagnostic"] == INSUFFICIENT_REDUNDANCY
                and value["temp_candidate_valid"]
            ),
            timeout=LONG_STATE_TIMEOUT_SECONDS,
        )
        for sensor in failed_sensors:
            assert degraded[f"sensor{sensor}_health"] == CHANNEL_READ_FAILURE
            assert not degraded[f"sensor{sensor}_valid"]
        assert degraded[f"sensor{remaining}_health"] == CHANNEL_HEALTHY
        assert degraded[f"sensor{remaining}_valid"]
        assert degraded["temp_candidate"] == degraded[f"temp{remaining}"]
        assert_instrumentation_invariants(degraded)
        record_robustness(
            name,
            {"baseline": baseline, "degraded": degraded},
            "yes",
            "yes; both explicit failed channels",
            f"non-authoritative Sensor {remaining} candidate",
            "no actuator decision is made by the simulated ESP32",
            "no",
        )
        restore_none(f"{name} recovery")


def test_robustness_correlated_bias():
    scenarios = (
        (
            "E1 correlated S1/S2 high bias",
            SENSOR1_SENSOR2_CORRELATED_BIAS,
            VOTE_PAIR_12,
            SENSOR_3_OUTLIER,
            3,
        ),
        (
            "E2 correlated S2/S3 high bias",
            SENSOR2_SENSOR3_CORRELATED_BIAS,
            VOTE_PAIR_23,
            SENSOR_1_OUTLIER,
            1,
        ),
    )

    for name, mode, vote, diagnostic, misisolated in scenarios:
        baseline = restore_none(f"{name} baseline")
        write_fault_mode(mode)
        selected = wait_for_snapshot(
            f"{name} majority selection",
            lambda value: (
                value["vote"] == vote
                and value["diagnostic"] == diagnostic
                and value["temp_candidate"] == baseline["temp_candidate"] + 500
            ),
            timeout=LONG_STATE_TIMEOUT_SECONDS,
        )
        assert all(
            selected[f"sensor{sensor}_health"] == CHANNEL_HEALTHY
            and selected[f"sensor{sensor}_valid"]
            for sensor in (1, 2, 3)
        )
        assert_instrumentation_invariants(selected)
        record_robustness(
            name,
            selected,
            "yes",
            f"no; healthy Sensor {misisolated} is isolated",
            "correlated biased majority is diagnostic evidence only",
            "no actuator decision is made by the simulated ESP32",
            "yes",
        )
        restore_none(f"{name} recovery")


def test_robustness_correlated_stuck():
    baseline = restore_none("F correlated S1/S2 stuck baseline")
    write_fault_mode(SENSOR1_SENSOR2_CORRELATED_STUCK)
    selected = wait_for_snapshot(
        "F correlated stuck majority selection",
        lambda value: (
            value["vote"] == VOTE_PAIR_12
            and value["diagnostic"] == SENSOR_3_OUTLIER
            and value["temp1"] == baseline["temp1"]
            and value["temp2"] == baseline["temp2"]
            and value["temp3"] > baseline["temp3"] + 100
        ),
        timeout=LONG_STATE_TIMEOUT_SECONDS,
    )
    assert selected["temp_candidate"] == baseline["temp_candidate"]
    assert all(
        selected[f"sensor{sensor}_health"] == CHANNEL_HEALTHY
        for sensor in (1, 2, 3)
    )
    assert_instrumentation_invariants(selected)
    record_robustness(
        "F correlated S1/S2 stuck",
        selected,
        "yes",
        "no; moving Sensor 3 is isolated as the outlier",
        "correlated frozen majority is diagnostic evidence only",
        "no actuator decision is made by the simulated ESP32",
        "yes",
    )
    restore_none("F correlated stuck recovery")


def test_robustness_correlated_drift():
    baseline = restore_none("G correlated S1/S2 drift baseline")
    write_fault_mode(SENSOR1_SENSOR2_CORRELATED_DRIFT)
    selected = wait_for_snapshot(
        "G correlated drift majority/stuck classification",
        lambda value: (
            value["vote"] == VOTE_PAIR_12
            and value["diagnostic"] == SENSOR_3_STUCK
            and value["temp1"] == value["temp2"]
            and value["temp1"] > value["temp3"] + 100
        ),
        timeout=LONG_STATE_TIMEOUT_SECONDS,
    )
    assert selected["temp_candidate"] == selected["temp1"]
    assert selected["temp3"] == baseline["temp3"]
    assert all(
        selected[f"sensor{sensor}_health"] == CHANNEL_HEALTHY
        for sensor in (1, 2, 3)
    )
    assert_instrumentation_invariants(selected)
    record_robustness(
        "G correlated S1/S2 drift",
        selected,
        "yes",
        "no; correct stationary Sensor 3 is declared stuck",
        "correlated drifting majority is diagnostic evidence only",
        "no actuator decision is made by the simulated ESP32",
        "yes",
    )
    restore_none("G correlated drift recovery")


def test_robustness_opposing_biases():
    baseline, pending, confirmed = capture_confirmed_unresolved(
        SENSOR1_HIGH_SENSOR2_LOW,
        "H S1 high plus S2 low",
    )
    assert all(
        confirmed[f"sensor{sensor}_health"] == CHANNEL_HEALTHY
        and confirmed[f"sensor{sensor}_valid"]
        for sensor in (1, 2, 3)
    )
    assert confirmed["temp1"] == baseline["temp1"] + 500
    assert confirmed["temp2"] == baseline["temp2"] - 500
    assert not confirmed["agree12"]
    assert not confirmed["agree13"]
    assert not confirmed["agree23"]
    record_robustness(
        "H independent opposing biases",
        {"pending": pending, "confirmed": confirmed},
        "yes",
        "ambiguous; no agreeing pair",
        "last diagnostic candidate while pending; none after confirmation",
        "safe",
        "yes",
    )
    restore_none("H opposing biases recovery")


def test_robustness_fault_during_recovery():
    restore_none("I fault-during-recovery baseline")
    write_fault_mode(SENSOR1_DISCONNECTED)
    first_fault = wait_for_snapshot(
        "I Sensor 1 initial fault",
        lambda value: (
            value["sensor1_health"] == CHANNEL_READ_FAILURE
            and value["sensor1_recovery"] == 0
        ),
    )
    write_fault_mode(NONE)
    recovering = wait_for_snapshot(
        "I Sensor 1 recovery sample 1",
        lambda value: (
            value["sensor1_health"] == CHANNEL_RECOVERING
            and value["sensor1_recovery"] == 1
            and not value["sensor1_valid"]
        ),
    )
    write_fault_mode(SENSOR2_DISCONNECTED)
    overlap = wait_for_snapshot(
        "I Sensor 2 fault while Sensor 1 recovers",
        lambda value: (
            value["sensor1_health"] == CHANNEL_RECOVERING
            and value["sensor1_recovery"] == 2
            and not value["sensor1_valid"]
            and value["sensor2_health"] == CHANNEL_READ_FAILURE
            and not value["sensor2_valid"]
            and value["sensor3_valid"]
            and value["vote"] == VOTE_SINGLE_3
            and value["diagnostic"] == INSUFFICIENT_REDUNDANCY
        ),
        timeout=LONG_STATE_TIMEOUT_SECONDS,
    )
    rejoined = wait_for_snapshot(
        "I Sensor 1 qualified while Sensor 2 remains failed",
        lambda value: (
            value["sensor1_health"] == CHANNEL_HEALTHY
            and value["sensor1_recovery"] == CHANNEL_RECOVERY_SAMPLES
            and value["sensor1_valid"]
            and value["sensor2_health"] == CHANNEL_READ_FAILURE
            and value["vote"] == VOTE_PAIR_13
        ),
        timeout=LONG_STATE_TIMEOUT_SECONDS,
    )
    assert_instrumentation_invariants(overlap)
    assert_instrumentation_invariants(rejoined)
    record_robustness(
        "I fault while another channel is recovering",
        {
            "initial_fault": first_fault,
            "recovering": recovering,
            "overlap": overlap,
            "rejoined": rejoined,
        },
        "yes",
        "yes; explicit channel faults remain distinct",
        "Sensor 3 alone, then Sensors 1/3 after qualification",
        "safe",
        "no",
    )
    restore_none("I fault-during-recovery recovery")


def test_robustness_simultaneous_recovery():
    restore_none("J simultaneous recovery baseline")
    write_fault_mode(BOTH_SENSORS_DISCONNECTED)
    failed = wait_for_snapshot(
        "J two-channel fault",
        lambda value: (
            value["sensor1_health"] == CHANNEL_READ_FAILURE
            and value["sensor2_health"] == CHANNEL_READ_FAILURE
            and value["sensor1_recovery"] == 0
            and value["sensor2_recovery"] == 0
            and value["vote"] == VOTE_SINGLE_3
        ),
    )
    write_fault_mode(NONE)
    first = wait_for_snapshot(
        "J simultaneous recovery sample 1",
        lambda value: all(
            value[f"sensor{sensor}_health"] == CHANNEL_RECOVERING
            and value[f"sensor{sensor}_recovery"] == 1
            and not value[f"sensor{sensor}_valid"]
            for sensor in (1, 2)
        ),
        timeout=LONG_STATE_TIMEOUT_SECONDS,
    )
    second, _ = next_sample(first["heartbeat"], "J recovery sample 2")
    assert all(
        second[f"sensor{sensor}_health"] == CHANNEL_RECOVERING
        and second[f"sensor{sensor}_recovery"] == 2
        and not second[f"sensor{sensor}_valid"]
        for sensor in (1, 2)
    ), second
    rejoined, _ = next_sample(second["heartbeat"], "J qualified re-entry")
    assert healthy(rejoined), rejoined
    assert_instrumentation_invariants(first)
    assert_instrumentation_invariants(second)
    assert_instrumentation_invariants(rejoined)
    record_robustness(
        "J simultaneous independent recovery",
        {
            "failed": failed,
            "first": first,
            "second": second,
            "rejoined": rejoined,
        },
        "yes",
        "yes; independent per-channel recovery counters",
        "Sensor 3 until both channels independently qualify",
        "safe",
        "no",
    )


def cleanup():
    errors = []
    try:
        write_fault_mode(NONE)
        wait_for_snapshot(
            "cleanup healthy NONE state",
            healthy,
            timeout=LONG_STATE_TIMEOUT_SECONDS,
        )
    except Exception as error:
        errors.append(f"fault mode: {error}")

    if errors:
        print(f"FAIL cleanup: {errors}")
    else:
        print("PASS cleanup: fault mode NONE")


def print_summary():
    passed = sum(succeeded for _, succeeded, _ in RESULTS)
    failed = len(RESULTS) - passed
    print("=" * 48)
    print("THREE-SENSOR VIRTUAL REGRESSION SUMMARY")
    print("=" * 48)
    for name, succeeded, _ in RESULTS:
        print(f"{'PASS' if succeeded else 'FAIL'} {name}")
    print(f"TOTAL: {passed} passed, {failed} failed")
    if failed == 0:
        print("ALL TESTS PASSED")

    if ROBUSTNESS_RESULTS:
        print("=" * 48)
        print("ROBUSTNESS CLASSIFICATION MATRIX")
        print("=" * 48)
        for name, classification in ROBUSTNESS_RESULTS:
            print(name)
            print(f"  Detection: {classification['detection']}")
            print(f"  Isolation: {classification['isolation']}")
            print(
                "  Accommodation: "
                f"{classification['accommodation']}"
            )
            print(f"  Safety result: {classification['safety']}")
            print(
                "  Architectural limitation: "
                f"{classification['architectural_limitation']}"
            )


def main():
    suite_error = None
    cases = [
        (
            "healthy operation",
            "three valid agreeing sensors and a three-sensor average",
            test_healthy_operation,
        ),
        (
            "constant fresh measurements",
            "unchanged values remain healthy when acquisitions are fresh",
            test_constant_fresh_measurements,
        ),
        (
            "Sensor 1 outlier",
            "Sensors 2 and 3 outvote biased Sensor 1",
            lambda: test_outlier(
                SENSOR1_BIAS,
                1,
                SENSOR_1_OUTLIER,
                VOTE_PAIR_23,
            ),
        ),
        (
            "Sensor 2 outlier",
            "Sensors 1 and 3 outvote biased Sensor 2",
            lambda: test_outlier(
                SENSOR2_BIAS,
                2,
                SENSOR_2_OUTLIER,
                VOTE_PAIR_13,
            ),
        ),
        (
            "Sensor 3 outlier",
            "Sensors 1 and 2 outvote biased Sensor 3",
            lambda: test_outlier(
                SENSOR3_BIAS,
                3,
                SENSOR_3_OUTLIER,
                VOTE_PAIR_12,
            ),
        ),
        (
            "Sensor 1 disconnect",
            "Sensors 2 and 3 remain valid as comparator evidence",
            lambda: test_disconnect(
                SENSOR1_DISCONNECTED,
                1,
                SENSOR_1_FAULT,
                VOTE_PAIR_23,
            ),
        ),
        (
            "Sensor 2 disconnect",
            "Sensors 1 and 3 remain valid as comparator evidence",
            lambda: test_disconnect(
                SENSOR2_DISCONNECTED,
                2,
                SENSOR_2_FAULT,
                VOTE_PAIR_13,
            ),
        ),
        (
            "Sensor 3 disconnect",
            "Sensors 1 and 2 remain valid as comparator evidence",
            lambda: test_disconnect(
                SENSOR3_DISCONNECTED,
                3,
                SENSOR_3_FAULT,
                VOTE_PAIR_12,
            ),
        ),
        (
            "Sensor 1 out of range",
            "Sensor 1 is range-invalid and rejoins after qualification",
            lambda: test_out_of_range(
                SENSOR1_OUT_OF_RANGE,
                1,
                SENSOR_1_FAULT,
                VOTE_PAIR_23,
            ),
        ),
        (
            "Sensor 2 out of range",
            "Sensor 2 is range-invalid and rejoins after qualification",
            lambda: test_out_of_range(
                SENSOR2_OUT_OF_RANGE,
                2,
                SENSOR_2_FAULT,
                VOTE_PAIR_13,
            ),
        ),
        (
            "Sensor 3 out of range",
            "Sensor 3 is range-invalid and rejoins after qualification",
            lambda: test_out_of_range(
                SENSOR3_OUT_OF_RANGE,
                3,
                SENSOR_3_FAULT,
                VOTE_PAIR_12,
            ),
        ),
        (
            "Sensor 1 stale acquisition",
            "Sensor 1 is excluded only after its refresh timeout",
            lambda: test_stale_acquisition(
                SENSOR1_STALE_ACQUISITION,
                1,
                SENSOR_1_FAULT,
                VOTE_PAIR_23,
            ),
        ),
        (
            "Sensor 2 stale acquisition",
            "Sensor 2 is excluded only after its refresh timeout",
            lambda: test_stale_acquisition(
                SENSOR2_STALE_ACQUISITION,
                2,
                SENSOR_2_FAULT,
                VOTE_PAIR_13,
            ),
        ),
        (
            "Sensor 3 stale acquisition",
            "Sensor 3 is excluded only after its refresh timeout",
            lambda: test_stale_acquisition(
                SENSOR3_STALE_ACQUISITION,
                3,
                SENSOR_3_FAULT,
                VOTE_PAIR_12,
            ),
        ),
        (
            "recovery interruption",
            "a renewed fault resets channel recovery qualification",
            test_recovery_interruption,
        ),
        (
            "degraded invalid counts",
            "one remaining sensor is explicitly degraded; zero are invalid",
            test_degraded_invalid_counts,
        ),
        (
            "temporary outlier",
            "one-shot Sensor 1 outlier re-arms only through NONE",
            test_temporary_outlier_rearm,
        ),
        (
            "persistent disagreement majority isolation",
            "legacy persistent disagreement becomes a stable Sensor 1 "
            "outlier under 2-out-of-3 voting",
            test_persistent_outlier,
        ),
        (
            "two-valid unresolved disagreement",
            "invalid Sensor 3 leaves disagreeing Sensors 1 and 2 unresolved",
            lambda: test_unresolved(
                SENSOR3_DISCONNECTED_SENSOR1_BIAS,
                "two-valid unresolved disagreement",
                True,
            ),
        ),
        (
            "three-way unresolved disagreement",
            "no agreeing pair invalidates the diagnostic candidate",
            lambda: test_unresolved(
                THREE_WAY_DISAGREEMENT,
                "three-way unresolved disagreement",
                False,
            ),
        ),
        (
            "Sensor 1 stuck phase 0",
            "Sensors 2 and 3 corroborate movement while Sensor 1 is flat",
            lambda: test_stuck(
                SENSOR1_STUCK,
                1,
                SENSOR_1_STUCK,
                VOTE_PAIR_23,
                0,
            ),
        ),
        (
            "Sensor 1 stuck phase 2",
            "stuck detection is independent of one window alignment",
            lambda: test_stuck(
                SENSOR1_STUCK,
                1,
                SENSOR_1_STUCK,
                VOTE_PAIR_23,
                2,
            ),
        ),
        (
            "Sensor 2 stuck",
            "Sensors 1 and 3 corroborate movement while Sensor 2 is flat",
            lambda: test_stuck(
                SENSOR2_STUCK,
                2,
                SENSOR_2_STUCK,
                VOTE_PAIR_13,
            ),
        ),
        (
            "Sensor 3 stuck",
            "Sensors 1 and 2 corroborate movement while Sensor 3 is flat",
            lambda: test_stuck(
                SENSOR3_STUCK,
                3,
                SENSOR_3_STUCK,
                VOTE_PAIR_12,
            ),
        ),
        (
            "common process change",
            "all sensors move together without a stuck false positive",
            test_common_process_change,
        ),
        (
            "drift versus stuck ambiguity",
            "stationary agreeing pair outvotes moving Sensor 1 without "
            "calling the pair stuck",
            test_drift_vs_stuck,
        ),
        (
            "invalid mode fail-safe",
            "unsupported mode is rewritten to NONE",
            test_invalid_mode,
        ),
        (
            "simulated responsibility boundary",
            "legacy physical commands cannot create a simulated ESP32 output",
            test_simulated_responsibility_boundary,
        ),
        (
            "robustness A channel failure plus outlier",
            "two usable disagreeing sensors remain unresolved",
            test_robustness_channel_failure_plus_outlier,
        ),
        (
            "robustness B channel failure plus stuck",
            "missing corroboration prevents a false stuck isolation",
            test_robustness_channel_failure_plus_stuck,
        ),
        (
            "robustness C out-of-range plus disagreement",
            "the remaining disagreeing pair remains unresolved",
            test_robustness_out_of_range_plus_disagreement,
        ),
        (
            "robustness D two explicit channel failures",
            "all single-sensor candidate permutations remain deterministic",
            test_robustness_two_explicit_channel_failures,
        ),
        (
            "robustness E correlated bias",
            "the correlated majority is selected deterministically",
            test_robustness_correlated_bias,
        ),
        (
            "robustness F correlated stuck",
            "the frozen majority can outvote the moving correct channel",
            test_robustness_correlated_stuck,
        ),
        (
            "robustness G correlated drift",
            "the drifting majority can misclassify the correct channel",
            test_robustness_correlated_drift,
        ),
        (
            "robustness H opposing biases",
            "three-way disagreement is confirmed without a candidate",
            test_robustness_opposing_biases,
        ),
        (
            "robustness I fault during recovery",
            "a recovering channel is not counted prematurely",
            test_robustness_fault_during_recovery,
        ),
        (
            "robustness J simultaneous recovery",
            "each failed channel completes its own qualification",
            test_robustness_simultaneous_recovery,
        ),
    ]

    try:
        for name, expected, operation in cases:
            run_case(name, expected, operation)
    except Exception as error:
        suite_error = error
    finally:
        cleanup()
        close_transport()
        print_summary()

    if suite_error is not None:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
