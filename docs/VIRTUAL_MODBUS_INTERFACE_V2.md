# Virtual Modbus Interface v2

**Status: FROZEN — final virtual implementation baseline (3 October 2026).**

This contract defines the clean Modbus TCP interface for the final virtual project.
The final TIA Portal project has been saved separately and the seven-register ESP32 → Modbus TCP → S7-1500/PLCSIM Advanced path has been commissioned successfully.

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

No virtual-project Modbus point gives the ESP32 authority over process state, trusted temperature, heater demand, permissives, trips, or the final heater output.

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
| 0 | HEALTHY | Channel may be usable if communication is fresh and the raw temperature is valid |
| 1 | READ_FAILURE | Channel unusable |
| 2 | OUT_OF_RANGE | Channel unusable |
| 3 | STALE | Channel unusable |
| 4 | STUCK | Channel unusable |
| 5 | RECOVERING | Channel temporarily unusable until ESP32 recovery qualification completes |
| 6 | NOT_READY | Channel has not yet completed initial qualification |

If more than one acquisition-side condition exists, the ESP32 publishes one deterministic dominant health state:

READ_FAILURE > OUT_OF_RANGE > STALE > STUCK > RECOVERING > NOT_READY > HEALTHY.

## Values deliberately removed from the operational interface

The following legacy values are not part of Virtual Modbus Interface v2:

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

The PLC already has all three temperatures and therefore owns pairwise agreement, trusted-temperature selection, redundancy classification, process state, trips, permissives and output. Recovery counters are internal acquisition implementation details, and physical-output objects do not belong to the final virtual build.

## PLC-side decoded data model

The final `Process_Data_DB` external-data model is limited to:

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

The frozen decoder preserves the verified baseline meaning of raw validity:

~~~text
TempN_Valid = (raw TempN <> 16#FFFF)
~~~

Health and communication freshness remain separate evidence. Final channel usability is therefore evaluated downstream using all three conditions:

~~~text
CommsOK AND TempN_Valid AND (SensorN_Health = HEALTHY)
~~~

This separation is intentional: raw-value validity, acquisition diagnostics, and communication freshness are distinct concepts.

## PLC logic retained after the interface refactor

The verified downstream PLC responsibilities remain unchanged:

1. Heartbeat_Watchdog
2. Trusted_Temperature_Selector
3. Evaluate_Temp_Quality
4. Process_State_Control
5. Heater_Permissive
6. Heater_Demand_Hysteresis
7. Calculate_Heater_Output

Only the Modbus receive/decode boundary changed from the legacy 19-register interface to the final seven-register interface.

## Test/commissioning interface

Fault injection remains available for commissioning but is not part of the operational interface.

| Address | Name | Purpose |
|---:|---|---|
| HR0 | FAULT_MODE | Select controlled firmware fault-injection scenario |

The PLC control program does not depend on `FAULT_MODE` during normal operation. No coils are required by the virtual project.

## Final commissioning state

The final virtual integration was commissioned with:

- Wokwi/ESP32 firmware running
- local Modbus test client successfully reading the seven-register server
- S7-PLCSIM Advanced running the saved TIA Portal project
- `MB_DATA_LEN = 7`
- receive buffer `Modbus_Data_DB.inputRegisters[0..6]`
- decoded temperatures, heartbeat and health values flowing into the PLC
- existing downstream verified control logic retained

During final commissioning, the PLC raw Modbus buffer initially did not update because only the software had been downloaded. Downloading both the TIA hardware/device configuration and software restored the working Modbus path. This is retained as a commissioning lesson: a successful software compile/download does not prove that the simulated CPU is running the current hardware/network configuration.

## Final safety invariant

The final simulated output authority remains:

~~~text
HEATER_OUTPUT = HEATER_DEMAND AND HEATER_PERMISSIVE
~~~

## Freeze rule

This v2 contract is now the frozen final virtual baseline. Further changes to register meanings, addresses, ownership boundaries, PLC execution order, diagnostic semantics or heater authority require an explicit new revision rather than silent edits to v2.
