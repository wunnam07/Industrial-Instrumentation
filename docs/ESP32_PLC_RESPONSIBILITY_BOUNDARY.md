# ESP32/PLC responsibility boundary

This contract applies to the active Wokwi/PLCSIM build.

## Ownership

~~~text
T1 / T2 / T3 / analog input
              |
              v
ESP32: acquisition, quality, freshness, diagnostic evidence, heartbeat
              |
              | Modbus TCP
              v
PLC: measurement acceptance, T_CONTROL, process state, permissives,
     trips, hysteresis, Auto/Manual, and simulated HEATER_OUTPUT
~~~

The ESP32 does not decide whether the simulated heater is on. Its comparator
may publish a temperature candidate for diagnostics and regression comparison,
but that value is not the PLC's authoritative control temperature.

## Stable Modbus address locations

Existing address locations are retained so consumers do not fail because
registers moved. Two legacy meanings are deliberately narrowed:

| Type/address | Active simulated meaning |
|---|---|
| Input register 0 | Sensor 1 temperature, scaled by 100; 65535 if invalid |
| Input register 1 | Sensor 2 temperature, scaled by 100; 65535 if invalid |
| Input register 2 | ESP32 diagnostic temperature candidate, scaled by 100; non-authoritative |
| Input register 3 | Absolute Sensor 1/Sensor 2 difference, scaled by 100 |
| Input registers 4–6 | Analog raw/scaled value and diagnostic status |
| Input register 7 | Legacy process state: always 65535 because the PLC owns process state |
| Input register 8 | ESP32 heartbeat; increments with each completed measurement cycle |
| Input register 9 | Sensor 3 temperature, scaled by 100; 65535 if invalid |
| Input registers 10–12 | Differences 1/3 and 2/3, then comparator/voting status |
| Input registers 13–18 | Per-channel health and recovery counters |
| Discrete input 0 | Instrument node ready after the first completed diagnostic cycle |
| Discrete inputs 1–3 | Sensor 1 validity, Sensor 2 validity, and agreement 1/2 |
| Discrete input 4 | Diagnostic temperature candidate valid |
| Discrete inputs 5–7 | Physical-output compatibility status; always false in the simulated build |
| Discrete inputs 8–11 | Sensor 3 validity, agreement 1/3, agreement 2/3, comparator resolved |

Holding register 1 remains the test-only fault-injection selector.

Coil 0 and holding register 0 retain their future physical-output address
locations. With ENABLE_PHYSICAL_HEATER_OUTPUT=0, writes are accepted for
compatibility but have no effect on GPIO, diagnostic values, or any simulated
heater output.

## Future physical-output build

The GPIO/MOSFET path is isolated behind ENABLE_PHYSICAL_HEATER_OUTPUT.
When explicitly enabled for a later physical prototype, the ESP32 can execute
the PLC's heater demand and reject it when the PLC heartbeat expires. The PLC
still owns temperature selection, process permissives, trips and demand.

The physical flag is disabled in the active esp32dev PlatformIO environment.
