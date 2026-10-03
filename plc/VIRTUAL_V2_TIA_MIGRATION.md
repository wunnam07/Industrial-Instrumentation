# TIA Portal migration — Virtual Modbus Interface v2

This migration changes only the ESP32/PLC data contract. The downstream control architecture remains the verified baseline unless compilation or testing proves otherwise.

## 1. Preserve the working project first

Create a copy/archive of the current TIA project before editing it. Treat that copy as the pre-v2 PLC baseline.

## 2. Replace the raw Modbus receive buffer

Use `Modbus_Data_DB_V2.scl` so the PLC raw buffer is:

```text
Modbus_Data_DB.inputRegisters[0..6] : ARRAY OF WORD
```

The seven words are:

| Index | Meaning |
|---:|---|
| 0 | TEMP1 x100; 16#FFFF invalid |
| 1 | TEMP2 x100; 16#FFFF invalid |
| 2 | TEMP3 x100; 16#FFFF invalid |
| 3 | ESP32 heartbeat |
| 4 | Sensor 1 health |
| 5 | Sensor 2 health |
| 6 | Sensor 3 health |

## 3. Change the existing MB_CLIENT read from 19 words to 7

Do not redesign the working connection configuration.

Keep the existing values that already worked for:

- Modbus TCP connection / CONNECT structure
- server IP and TCP port
- MB_MODE / input-register function selection
- base Modbus data address
- request-trigger logic
- BUSY/DONE/ERROR handling

Change only the operational data length from **19** input registers to **7** and make the receive target the new `Modbus_Data_DB.inputRegisters[0..6]` array.

In the existing MB_CLIENT call, this is the parameter that previously requested 19 registers (`MB_DATA_LEN` in the current Siemens block interface).

Do not change the base address merely because the firmware uses zero-based internal offsets. Preserve the base address that already produced the correct old TEMP1 value at `inputRegisters[0]`.

## 4. Replace the old process-data decoder

Import/replace with `Process_Data_DB_and_Decode_V2.scl`.

The new decoded external-data model is intentionally limited to:

```text
Temp1_C
Temp2_C
Temp3_C
Temp1_Valid
Temp2_Valid
Temp3_Valid
ESP32_Heartbeat
Sensor1_Health
Sensor2_Health
Sensor3_Health
```

The removed PLC fields are not missing functionality. They were either obsolete, redundant, test-only, or calculations now owned elsewhere in the PLC.

## 5. Keep the existing downstream execution order

Use the verified order:

```text
Cyclic Modbus request generation
MB_CLIENT
Decode_Process_Data
Heartbeat_Watchdog
Trusted_Temperature_Selector
Evaluate_Temp_Quality
Process_State_Control
Heater_Permissive
Heater_Demand_Hysteresis
Calculate_Heater_Output
```

## 6. OB1 signal wiring that should remain

`Heartbeat_Watchdog`:

```text
Heartbeat := Process_Data_DB.ESP32_Heartbeat
Timeout   := T#3s
```

`Trusted_Temperature_Selector`:

```text
Temp1_C      := Process_Data_DB.Temp1_C
Temp2_C      := Process_Data_DB.Temp2_C
Temp3_C      := Process_Data_DB.Temp3_C
Temp1Valid   := Process_Data_DB.Temp1_Valid
Temp2Valid   := Process_Data_DB.Temp2_Valid
Temp3Valid   := Process_Data_DB.Temp3_Valid
NewDataPulse := Heartbeat_Watchdog_DB.HeartbeatChanged
CommsOK      := Heartbeat_Watchdog_DB.CommsOK
```

The v2 decoder marks a channel valid only when its register is not `16#FFFF` and ESP32 health is `HEALTHY`. Therefore the trusted selector receives acquisition-qualified measurements, while it still owns cross-sensor agreement and trusted-temperature selection.

`Evaluate_Temp_Quality` can remain unchanged. Its additional health checks are redundant with v2 `TempN_Valid`, but are safe and provide explicit defensive validation. Do not rewrite a verified block only to remove that redundancy before FAT.

The remaining Process_State, heater permissive, hysteresis and final-output wiring remains unchanged.

## 7. Removed dependencies — check for compile references

After replacing `Process_Data_DB`, search the TIA project for these old fields and remove only genuine remaining references:

```text
TempCandidate_C
TempCandidate_Valid
SensorDiff12_C
SensorDiff13_C
SensorDiff23_C
SensorDiff12_Valid
SensorDiff13_Valid
SensorDiff23_Valid
PotRaw
PotScaled_pct
DiagnosticStatus
LegacyProcessStateRaw
VotingStatus
Sensor1_Recovery
Sensor2_Recovery
Sensor3_Recovery
```

The verified downstream blocks should not require them. Any remaining reference is evidence of a hidden legacy dependency and must be reviewed rather than blindly replaced.

## 8. Compile gate

Before downloading to PLCSIM:

- Compile software blocks.
- Require zero compile errors.
- Do not mark the v2 PLC migration complete if TIA silently recreated an old DB structure or if any removed field is still referenced.

## 9. Minimal online commissioning sequence

With Wokwi v2 running and PLCSIM connected:

1. Confirm `inputRegisters[0..6]` change online.
2. Confirm healthy temperatures decode into `Temp1_C`, `Temp2_C`, `Temp3_C`.
3. Confirm the three health values are `0` after startup qualification.
4. Confirm `ESP32_Heartbeat` changes and `CommsOK = TRUE`.
5. Confirm trusted-temperature selection is valid with three agreeing channels.
6. Inject one channel disconnect and verify that channel becomes invalid while the other two remain available.
7. Inject a bias/disagreement and verify the biased channel can remain acquisition-healthy while PLC disagreement logic handles the measurement conflict.
8. Inject stale/stuck/out-of-range cases and verify ESP32 health excludes the affected channel.
9. Stop Wokwi/communications and verify PLC communication-loss behavior.
10. Re-run warning, trip, reset, heater-permissive, hysteresis and final-output tests.

## 10. Acceptance criterion

The migration is accepted only when the end-to-end system still proves:

```text
HEATER_OUTPUT = HEATER_DEMAND AND HEATER_PERMISSIVE
```

and all previously verified safety/state behaviors still pass under the new seven-register interface.

Until those tests pass, `virtual-v2-cleanup` is a migration branch, not the frozen final virtual baseline.
