# ESP32/PLC responsibility boundary — virtual project

This file describes the final virtual architecture on the `virtual-v2-cleanup` branch. The detailed register contract is in `docs/VIRTUAL_MODBUS_INTERFACE_V2.md`.

## Ownership rule

~~~text
DS18B20 A / B / C
        |
        v
ESP32
- acquire the three temperature channels
- detect acquisition/read/range/freshness/stuck health
- qualify channel recovery
- publish an application heartbeat
        |
        | Modbus TCP
        v
S7-1500 PLC
- supervise communication freshness
- decide PLC-side channel usability
- compare channels / determine agreement
- select trusted temperature
- classify temperature quality
- own process state
- own warning, trip, reset and interlocks
- own heater demand and permissive
- own simulated HEATER_OUTPUT
~~~

The design rule is:

> ESP32 publishes measurements and acquisition evidence. PLC makes control decisions.

## Virtual operational interface

The virtual build exposes exactly seven operational Modbus input registers:

| Offset | Meaning |
|---:|---|
| 0 | Temperature channel A |
| 1 | Temperature channel B |
| 2 | Temperature channel C |
| 3 | ESP32 heartbeat |
| 4 | Channel A health |
| 5 | Channel B health |
| 6 | Channel C health |

There are no virtual-project coils and no physical-heater command/status objects.

One holding register at offset 0 is reserved for controlled fault injection during test/commissioning. It is not part of normal PLC control operation.

## Deliberately removed from the virtual boundary

The ESP32 no longer publishes a trusted/candidate process temperature, pairwise differences, voting result, global process state, potentiometer values, recovery counters, agreement discrete inputs, or any physical-heater compatibility objects.

Those values either duplicate information the PLC already owns, expose ESP32 implementation details that the PLC does not need, or belong only to a future physical extension.

## Physical extension

The pre-refactor broader firmware is preserved on the `physical-capable-baseline` branch. A later physical project should be developed separately from the clean virtual baseline and may add a PLC-to-ESP32 final output command plus PLC heartbeat/freshness supervision.

The PLC remains the sole authority for trusted temperature, process state, trips, permissives, heater demand and final output decision.
