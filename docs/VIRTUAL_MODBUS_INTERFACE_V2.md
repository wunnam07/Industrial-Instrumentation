# Virtual Modbus Interface v2

This contract defines the clean Modbus TCP interface for the final virtual project.

## Design principle

The ESP32 publishes measurements and acquisition-level diagnostic evidence.
The PLC owns interpretation and control decisions.

~~~text
DS18B20 A/B/C
      |
      v
ESP32
- acquisition
- range/read/freshness/stuck diagnostics
- heartbeat
      |
      | Modbus TCP
      v
S7-1500 PLC
- communication supervision
- channel usability
- pairwise agreement
- trusted-temperature selection
- temperature quality
- process state
- warning/trip/reset
- heater demand
- heater permissive
- simulated HEATER_OUTPUT
~~~

No virtual-project Modbus point may give the ESP32 authority over process state, trusted temperature, heater demand, permissives, trips, or the final heater output.

## Operational Modbus input-register map

All addresses below are zero-based Modbus input-register offsets as used by the firmware library and PLC raw buffer.

| Address | Name | Encoding | Meaning |
|---:|---|---|---|
| 0 | TEMP1 | UINT16, temperature x100; 65535 invalid | DS18B20 channel A measurement |
| 1 | TEMP2 | UINT16, temperature x100; 65535 invalid | DS18B20 channel B measurement |
| 2 | TEMP3 | UINT16, temperature x100; 65535 invalid | DS18B20 channel C measurement |
| 3 | ESP32_HEARTBEAT | UINT16 incrementing counter | Fresh completed acquisition/diagnostic cycle evidence |
| 4 | SENSOR1_HEALTH | UINT16 enum | Acquisition/diagnostic state for channel A |
| 5 | SENSOR2_HEALTH | UINT16 enum | Acquisition/diagnostic state for channel B |
| 6 | SENSOR3_HEALTH | UINT16 enum | Acquisition/diagnostic state for channel C |

The PLC reads exactly seven operational input registers.

## Per-channel health enumeration

The health value is acquisition-side evidence, not a process-state or voting decision.

| Value | State | PLC interpretation |
|---:|---|---|
| 0 | HEALTHY | Channel may be considered usable if communication is fresh and the temperature register is not 65535 |
| 1 | READ_FAILURE | Channel unusable |
| 2 | OUT_OF_RANGE | Channel unusable |
| 3 | STALE | Channel unusable |
| 4 | STUCK | Channel unusable |
| 5 | RECOVERING | Channel temporarily unusable until ESP32 recovery qualification completes |
| 6 | NOT_READY | Channel has not yet completed initial qualification |

If more than one acquisition-side condition exists, the ESP32 shall publish one deterministic dominant health state. Recommended priority is:

READ_FAILURE > OUT_OF_RANGE > STALE > STUCK > RECOVERING > NOT_READY > HEALTHY.

The exact priority must be implemented once and tested; the PLC must not attempt to reconstruct ESP32 diagnostic state from raw history.

## Values deliberately removed from the operational interface

The following current/legacy values are not part of Virtual Modbus Interface v2:

- ESP32 temperature candidate
- pairwise temperature-difference registers
- potentiometer raw/scaled values
- global DiagnosticStatus
- legacy ESP32 process state
- ESP32 voting status
- channel recovery counters
- discrete-input validity/agreement/voting objects
- physical heater permissive/output/comms status
- physical heater demand coil
- physical PLC heartbeat holding register

Reasons:

- The PLC already has all three temperatures and therefore owns pairwise agreement, trusted-temperature selection, redundancy classification, process state, trips, permissives and output.
- Recovery counters are internal acquisition implementation details; the PLC only needs the resulting channel health state.
- Physical-output objects belong to a future physical project, not the final virtual build.

## PLC-side decoded data model

The replacement Process_Data_DB for the virtual project should contain only external data that crosses the ESP32/PLC boundary plus decoded validity flags:

~~~text
Temp1_C : Real
Temp2_C : Real
Temp3_C : Real

Temp1_Valid : Bool
Temp2_Valid : Bool
Temp3_Valid : Bool

ESP32_Heartbeat : UInt

Sensor1_Health : UInt
Sensor2_Health : UInt
Sensor3_Health : UInt
~~~

Recommended validity rule:

~~~text
TempN_Valid = (raw TempN <> 16#FFFF) AND (SensorN_Health = HEALTHY)
~~~

Communication freshness remains a separate PLC responsibility through Heartbeat_Watchdog. A channel is therefore usable only when:

~~~text
CommsOK AND TempN_Valid
~~~

## PLC logic retained after the interface refactor

The following downstream PLC responsibilities remain conceptually unchanged and should not be rewritten unless compilation/testing proves an interface dependency:

1. Heartbeat_Watchdog
2. Trusted_Temperature_Selector
3. Evaluate_Temp_Quality
4. Process_State_Control
5. Heater_Permissive
6. Heater_Demand_Hysteresis
7. Calculate_Heater_Output

Only their input wiring should be adjusted where Process_Data_DB fields change.

## Test/commissioning interface

Fault injection remains useful for virtual FAT but is not part of the operational interface.

For the virtual test build, reserve one test-only holding register:

| Address | Name | Purpose |
|---:|---|---|
| 0 | FAULT_MODE | Select controlled firmware fault-injection scenario |

This holding register shall be clearly documented as commissioning/test only. The PLC control program does not depend on it during normal operation.

No coils are required by the virtual project.

## Future physical project

The physical extension should start from the clean measurement interface above and add a separate PLC-to-ESP32 output command contract. The virtual project must not carry dormant physical-output objects.

A future physical extension may add, for example:

- final PLC HEATER_OUTPUT_CMD
- PLC heartbeat/freshness command
- hardware-output execution status

The PLC must still own trusted temperature, process state, trips, permissives, demand and final output authority. The ESP32 physical-output role is execution plus stale-command fail-safe shutdown.

## Migration acceptance criteria

The v2 interface is accepted only after all of the following are demonstrated:

- normal three-sensor acquisition
- one-sensor failure and recovery
- out-of-range channel
- stale channel
- stuck-channel diagnostic
- sensor disagreement handled by PLC logic
- communication loss and recovery
- trusted-temperature selection
- temperature-quality classification
- warning entry/clear
- high-temperature trip
- reset acceptance/rejection
- heater permissive behavior
- hysteresis demand behavior
- final invariant: HEATER_OUTPUT = HEATER_DEMAND AND HEATER_PERMISSIVE

After the tests pass, the virtual firmware, PLC decode SCL, I/O & Interface List, Modbus map and final documentation must all reference this same v2 contract.
