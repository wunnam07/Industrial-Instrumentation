# Final Virtual Implementation Baseline

**Frozen:** 3 October 2026  
**Branch:** `virtual-v2-cleanup`  
**Implementation parent before freeze documentation:** `b7da8a86b491403121f258d99a94639bf3db9e10`

## Scope

This file marks the final virtual implementation of the Industrial Instrumentation project as frozen.

The frozen system is:

```text
3 × virtual DS18B20 temperature channels
        ↓
ESP32 / Wokwi
        ↓
Modbus TCP
        ↓
S7-1500 / S7-PLCSIM Advanced
        ↓
PLC communication supervision
        ↓
channel usability + sensor agreement
        ↓
trusted-temperature selection
        ↓
process state / warning / trip / reset
        ↓
heater permissive + hysteresis demand
        ↓
HEATER_OUTPUT (simulated PLC output only)
        ↓
WinCC supervision
```

No physical heater or final control element is part of this frozen project scope.

## Frozen ESP32/PLC contract

Operational Modbus input registers are exactly:

| Offset | Signal |
|---:|---|
| 0 | TEMP1 |
| 1 | TEMP2 |
| 2 | TEMP3 |
| 3 | ESP32_HEARTBEAT |
| 4 | SENSOR1_HEALTH |
| 5 | SENSOR2_HEALTH |
| 6 | SENSOR3_HEALTH |

`HR0 FAULT_MODE` is commissioning/test only and is not part of the operational control interface.

The PLC raw receive buffer is:

```text
Modbus_Data_DB.inputRegisters[0..6]
```

The PLC decoder preserves raw validity semantics:

```text
TempN_Valid = (raw TempN <> 16#FFFF)
```

Final usability additionally requires fresh communication and `SensorN_Health = HEALTHY`.

## Frozen PLC responsibility boundary

The ESP32 owns:

- DS18B20 acquisition
- read/range/freshness/stuck diagnostics
- recovery qualification
- heartbeat publication

The PLC owns:

- communication supervision
- sensor usability
- pairwise agreement
- trusted-temperature selection
- temperature-quality classification
- process state
- warning/trip/reset behavior
- heater demand
- heater permissive
- final simulated output

## Frozen execution order

1. Cyclic Modbus request generation
2. MB_CLIENT
3. Decode_Process_Data
4. Heartbeat_Watchdog
5. Trusted_Temperature_Selector
6. Evaluate_Temp_Quality
7. Process_State_Control
8. Heater_Permissive
9. Heater_Demand_Hysteresis
10. Calculate_Heater_Output

## Final control constants

```text
T_SP        = 40 °C
T_ON        = 38 °C
T_OFF       = 42 °C
T_WARNING   = 50 °C
T_TRIP      = 60 °C
T_VALID_MIN = 0 °C
T_VALID_MAX = 80 °C
```

Hysteresis behavior:

- heater demand ON at or below 38 °C
- heater demand OFF at or above 42 °C
- previous demand retained between 38 °C and 42 °C

## Final safety invariant

```text
HEATER_OUTPUT = HEATER_DEMAND AND HEATER_PERMISSIVE
```

## Commissioning state

The final seven-register integration is operational end-to-end. The Wokwi ESP32 server was independently readable from the local Modbus test client, and the saved TIA Portal project successfully received the seven-register data after both the updated hardware/device configuration and software were downloaded to S7-PLCSIM Advanced.

The final TIA Portal project has been saved separately by the project owner.

## Freeze policy

This baseline is closed to silent functional changes. Any later change to register addresses, signal meanings, diagnostic enums, ownership boundaries, PLC execution order, thresholds, trip/reset behavior, permissives, or output authority must be made as an explicit new revision rather than modifying the frozen v2 baseline in place.
