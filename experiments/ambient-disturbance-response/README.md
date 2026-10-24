# IC-CPS — Ambient-Temperature Disturbance Response

A **standalone, virtual** modelling-and-control experiment for the IC-CPS learning project, corresponding to **Section E — apply an ambient-temperature disturbance and measure deviation/recovery** of the Notion completion checklist.

> **Boundary:** This is not a revision to the frozen `virtual-v2-cleanup` industrial instrumentation baseline. It does not communicate with the ESP32, Modbus, PLC or WinCC. Physical tests have **not** been performed.

## Engineering question

If the ambient temperature falls from **25°C to 20°C**, can a **100 W ON/OFF heater** still regulate a modelled thermal mass between **38°C and 42°C**? How do its heating time, cooling time, switching rate and duty cycle change? What happens **during** the disturbance, rather than only before and after?

## Physical model

Conservation of thermal energy gives

\[
C\frac{dT}{dt}=Pu-H(T-T_a).
\]

The code applies **Forward Euler**:

\[
T_{k+1}=T_k+\Delta t\frac{Pu_k-H(T_k-T_{a,k})}{C}.
\]

| Symbol | Meaning | Value | Unit |
|---|---|---:|---|
| `C` | lumped thermal capacitance | 800 | J/K |
| `H` | heat-loss coefficient | 4 | W/K |
| `P` | heater electrical power when ON | 100 | W |
| `T_a` | ambient temperature | 25 → 20 | °C |
| `T_ON` | turn heater ON when `T ≤ T_ON` | 38 | °C |
| `T_OFF` | turn heater OFF when `T ≥ T_OFF` | 42 | °C |
| `T(0)` | initial process temperature | 38 | °C |
| `τ = C/H` | thermal time constant | 200 | s |
| `dt` | numerical timestep and *current experiment's* controller evaluation interval | 0.1 | s |

At `t = 1200 s` (20 min), the ambient steps to 20°C. The simulation ends at `t = 3600 s` (60 min). The state temperature is **not reset** when the disturbance occurs. Only the heat-loss term changes.

### Assumptions and limitations

- One uniform process temperature (lumped model).
- Constant `C`, `H`, and heater power; instantaneous heater switching.
- Ambient is prescribed, not dynamically modelled.
- No sensor noise, sampling delay distinct from numerical `dt`, thermal transport delay, heater inertia, saturation beyond ON/OFF, or power electronics losses.
- The heater command and plant are evaluated at the **same** 0.1 s interval here for simplicity. The controller sampling experiment is a separate later checklist item.
- A real hysteresis-controlled plant normally forms a **limit cycle**, not a fixed equilibrium at 40°C. A disturbance can change that periodic cycle without causing an out-of-band excursion.
- This is model verification only; do not interpret it as measured plant performance or safety certification.

## Run

Use Python 3.10+:

```bash
python -m pip install -r requirements.txt
python thermal_disturbance.py
```

To run tests:

```bash
python -m pip install pytest
python -m pytest -q
```

Use finer integration to assess convergence:

```bash
python thermal_disturbance.py --dt 0.01 --output-dir results_fine
```

To change the ambient step:

```bash
python thermal_disturbance.py --before 25 --after 20 \
  --disturbance-at 1200 --duration 3600 --dt 0.1
```

### Generated outputs

- `results/disturbance_response.png` — aligned temperature, heater command, ambient disturbance plots.
- `results/timeseries.csv` — time, process temperature, ambient temperature, heater command and heating rate. Each row represents the **start** of an interval, so the final endpoint is not included.
- `results/summary.json` — physical parameters, exact cycle predictions, simulated cycle measurements and disturbance-instant slope change.

To avoid distorted statistics, the program measures only **complete OFF-to-OFF cycles** fully contained before or after the ambient step, excluding any cycle spanning the change. In each complete cycle, OFF cooling is followed by ON heating. The first/last truncated cycles are also excluded.

## Analytical predictions

For constant ambient, `T_eq,ON = T_a + P/H` and `τ = C/H`. With heating from 38°C to 42°C:

\[
t_{ON}=\tau\ln\left(\frac{T_{eq,ON}-38}{T_{eq,ON}-42}\right).
\]

With cooling from 42°C to 38°C (heater OFF):

\[
t_{OFF}=\tau\ln\left(\frac{42-T_a}{38-T_a}\right).
\]

Cycle duty and average power follow:

\[
D=\frac{t_{ON}}{t_{ON}+t_{OFF}},\qquad \bar P=P D.
\]

The program prints numerical averages beside these exact predictions. Expect approximately:

| Quantity | Before (25°C) | After (20°C) |
|---|---:|---:|
| ON equilibrium | 50°C | 45°C |
| Heating time | 81.09 s | 169.46 s |
| Cooling time | 53.65 s | 40.13 s |
| Duty cycle | 60.18% | 80.85% |

A lower ambient increases losses, so heating takes longer, cooling is faster and the heater runs for a larger fraction of time. At a **fixed** process temperature and heater command, the slope changes by

\[
\Delta\dot T=\frac{H(T_{a,after}-T_{a,before})}{C}
=\frac{4(20-25)}{800}=-0.025\;\mathrm{°C/s}.
\]

**Interpretation of recovery:** This ideal model continues crossing 38°C and 42°C after the disturbance. There is no unique "return-to-40°C settling time" under ON/OFF hysteresis. Instead, examine whether temperatures remain within the operating band and when a full cycle under the new ambient is observed.

## Check the implementation, don't treat it as a black box

1. From the initial state `T[0] = 38°C`, predict the first heating rate: `(100 - 4*(38 - 25))/800 = +0.060 °C/s`. With `dt=0.1 s`, the next temperature must be `38.006°C`.
2. At the disturbance instant, check that temperature has **no artificial 5°C jump**, but heating/cooling slope is 0.025°C/s lower at the same process temperature and heater state.
3. Compare the measured complete-cycle ON/OFF durations with the exact formulas. Differences should be consistent with switching-grid discretization.
4. Run `dt = 0.1` and `dt = 0.01` and compare duty cycles, min/max temperatures and cycle periods.
5. Explain why lower switching frequency can coexist with **higher** energy use per hour.

## Project roadmap

Current **CORE** task: disturbance-response analysis. Not included in this experiment: separate controller sampling-time study, systematic variation of `C/H/P`, PLC/ESP32 end-to-end plant integration, or physical validation.
