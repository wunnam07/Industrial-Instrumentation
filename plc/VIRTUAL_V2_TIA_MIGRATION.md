# TIA Portal migration — Virtual Modbus Interface v2

**Status: COMPLETE — incorporated into the frozen final virtual baseline on 3 October 2026.**

This migration changed only the ESP32/PLC data contract. The downstream control architecture remained the verified baseline.

## Final raw Modbus receive buffer

The PLC raw buffer is:

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

## Final MB_CLIENT change

The working Modbus TCP connection configuration was preserved. The operational receive length was changed from 19 words to 7 and the receive target is the new `Modbus_Data_DB.inputRegisters[0..6]` array.

The existing working values were retained for:

- Modbus TCP connection / CONNECT structure
- server IP and TCP port
- MB_MODE / input-register function selection
- base Modbus data address
- request-trigger logic
- BUSY/DONE/ERROR handling

The base Modbus data address was not changed merely because firmware offsets are documented from zero.

## Final decoded process-data model

`Process_Data_DB_and_Decode_V2.scl` provides:

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

The frozen decoder preserves the previously verified raw-validity semantics:

```text
TempN_Valid := raw TempN <> 16#FFFF
```

Sensor health and communication freshness remain separate evidence. `Evaluate_Temp_Quality` combines them downstream, so a channel is usable only when communication is healthy, the raw temperature is valid, and the corresponding ESP32 health state is `HEALTHY`.

## Retained execution order

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

## Retained OB1 signal wiring

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

The Process_State, heater permissive, hysteresis and final-output wiring remains unchanged from the verified baseline.

## Removed legacy dependencies

The final v2 process-data model no longer exposes:

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

These were obsolete, redundant, test-only, or calculations already owned elsewhere in the PLC.

## Final commissioning result

The migrated PLC project compiled with zero errors and was downloaded to S7-PLCSIM Advanced. During commissioning, the raw Modbus receive buffer initially remained static even though the Wokwi server and local Python Modbus client were working.

The cause was deployment state rather than the v2 register contract: the updated hardware/device configuration had not been downloaded together with the software. After downloading both **hardware and software**, `Modbus_Data_DB.inputRegisters[0..6]` updated correctly and the ESP32 values were visible in the PLC again.

This establishes the final commissioning rule for this project:

> After changes that affect the PLC device/network configuration, verify that both hardware configuration and software are downloaded to the simulated CPU. A successful software-only compile/download is not sufficient evidence that the running PLCSIM instance matches the offline TIA project.

## Final acceptance state

The seven-register ESP32 → Modbus TCP → S7-1500/PLCSIM Advanced interface is operational, the downstream verified control architecture is retained, and the TIA Portal project has been saved as the final implementation.

The final safety invariant remains:

```text
HEATER_OUTPUT = HEATER_DEMAND AND HEATER_PERMISSIVE
```

`virtual-v2-cleanup` is therefore no longer treated as an in-progress migration branch; it is the frozen final virtual implementation baseline.
