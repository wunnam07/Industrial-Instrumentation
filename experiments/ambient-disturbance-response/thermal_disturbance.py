"""IC-CPS Step 3: first-order thermal plant with an ambient-temperature step.

The plant is simulated with Forward Euler:
    C * dT/dt = P*u - H*(T - Ta)
    T[k+1] = T[k] + dt*(P*u[k] - H*(T[k] - Ta[k]))/C

Heater demand uses 38/42 deg C hysteresis; u=1 means ON.
This is a *virtual plant experiment*, not a connection to the frozen PLC project.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import matplotlib
matplotlib.use("Agg")  # Save plots without requiring a graphical desktop.
import matplotlib.pyplot as plt
import numpy as np


@dataclass(frozen=True)
class Settings:
    power_w: float = 100.0               # Electrical heater power when ON [W]
    capacitance_j_per_k: float = 800.0    # C = H*tau = 4*200 [J/K]
    heat_loss_w_per_k: float = 4.0        # H [W/K]
    heater_on_c: float = 38.0             # ON when T <= 38 deg C
    heater_off_c: float = 42.0            # OFF when T >= 42 deg C
    ambient_before_c: float = 25.0
    ambient_after_c: float = 20.0
    initial_temperature_c: float = 38.0
    disturbance_at_s: float = 1200.0
    total_time_s: float = 3600.0
    dt_s: float = 0.1

    @property
    def tau_s(self) -> float:
        return self.capacitance_j_per_k / self.heat_loss_w_per_k

    def validate(self) -> None:
        if min(self.power_w, self.capacitance_j_per_k,
               self.heat_loss_w_per_k, self.dt_s) <= 0:
            raise ValueError("P, C, H, and dt must all be positive.")
        if self.heater_on_c >= self.heater_off_c:
            raise ValueError("The ON threshold must be below the OFF threshold.")
        if not 0 < self.disturbance_at_s < self.total_time_s:
            raise ValueError("Disturbance must occur strictly within the simulation.")
        for value, name in ((self.total_time_s, "total_time_s"),
                            (self.disturbance_at_s, "disturbance_at_s")):
            count = round(value / self.dt_s)
            if not math.isclose(count * self.dt_s, value, abs_tol=1e-8):
                raise ValueError(f"{name} must be an integer multiple of dt_s.")


@dataclass
class Simulation:
    time_s: np.ndarray          # N+1 time edges
    temperature_c: np.ndarray   # N+1 states
    ambient_c: np.ndarray       # N+1 ambient temperatures
    heater_on: np.ndarray       # N commands on intervals [t[k], t[k+1])
    heating_rate_c_per_s: np.ndarray  # N slopes for the same intervals


@dataclass(frozen=True)
class Cycle:
    """A full OFF-to-OFF cycle, i.e. OFF cooling followed by ON heating."""
    start_s: float
    end_s: float
    on_s: float
    off_s: float
    period_s: float
    duty_percent: float
    min_c: float
    max_c: float


def simulate(settings: Settings) -> Simulation:
    """Integrate the physical model and hysteresis logic step by step."""
    settings.validate()
    n = round(settings.total_time_s / settings.dt_s)
    t = np.arange(n + 1, dtype=float) * settings.dt_s
    ambient = np.where(t < settings.disturbance_at_s,
                       settings.ambient_before_c, settings.ambient_after_c)
    temp = np.empty(n + 1, dtype=float)
    heater = np.empty(n, dtype=np.int8)
    rate = np.empty(n, dtype=float)
    temp[0] = settings.initial_temperature_c
    heater_state = 0

    for k in range(n):
        # Controller sees T[k]; within the deadband it holds its previous state.
        if temp[k] <= settings.heater_on_c:
            heater_state = 1
        elif temp[k] >= settings.heater_off_c:
            heater_state = 0
        heater[k] = heater_state

        # The ambient step changes the slope, not the stored temperature.
        rate[k] = (settings.power_w * heater_state
                   - settings.heat_loss_w_per_k * (temp[k] - ambient[k])
                   ) / settings.capacitance_j_per_k
        temp[k + 1] = temp[k] + settings.dt_s * rate[k]

    return Simulation(t, temp, ambient, heater, rate)


def full_cycles(result: Simulation, dt_s: float) -> list[Cycle]:
    """Measure only completed OFF-to-OFF cycles; exclude partial cycles."""
    u = result.heater_on
    off_events = np.flatnonzero((u[:-1] == 1) & (u[1:] == 0)) + 1
    cycles = []
    for a, b in zip(off_events[:-1], off_events[1:]):
        sequence = u[a:b]
        # Ensure that exactly one OFF->ON transition appears in this cycle.
        on_transitions = np.count_nonzero((sequence[:-1] == 0) &
                                          (sequence[1:] == 1))
        if on_transitions != 1:
            continue
        on_s = float(np.count_nonzero(sequence == 1) * dt_s)
        off_s = float(np.count_nonzero(sequence == 0) * dt_s)
        period_s = float((b - a) * dt_s)
        cycles.append(Cycle(
            start_s=float(result.time_s[a]), end_s=float(result.time_s[b]),
            on_s=on_s, off_s=off_s, period_s=period_s,
            duty_percent=100.0 * on_s / period_s,
            min_c=float(np.min(result.temperature_c[a:b + 1])),
            max_c=float(np.max(result.temperature_c[a:b + 1])),
        ))
    return cycles


def exact_cycle(settings: Settings, ambient_c: float) -> dict[str, float]:
    """Analytical predictions for a *constant-ambient*, complete limit cycle."""
    eq_on = ambient_c + settings.power_w / settings.heat_loss_w_per_k
    if not (ambient_c < settings.heater_on_c and
            eq_on > settings.heater_off_c):
        raise ValueError("Cannot complete both threshold crossings at this ambient.")
    on_s = settings.tau_s * math.log(
        (eq_on - settings.heater_on_c) / (eq_on - settings.heater_off_c))
    off_s = settings.tau_s * math.log(
        (settings.heater_off_c - ambient_c) / (settings.heater_on_c - ambient_c))
    period_s = on_s + off_s
    return {
        "on_s": on_s,
        "off_s": off_s,
        "period_s": period_s,
        "cycles_per_hour": 3600.0 / period_s,
        "duty_percent": 100.0 * on_s / period_s,
        "average_power_w": settings.power_w * on_s / period_s,
        "equilibrium_on_c": eq_on,
    }


def average_cycles(cycles: list[Cycle], power_w: float) -> dict[str, float | int]:
    if not cycles:
        raise ValueError("No complete cycles available for this operating regime.")
    avg = lambda name: float(np.mean([getattr(c, name) for c in cycles]))
    on_s, off_s, period_s = avg("on_s"), avg("off_s"), avg("period_s")
    return {
        "number_of_cycles": len(cycles),
        "on_s": on_s,
        "off_s": off_s,
        "period_s": period_s,
        "cycles_per_hour": 3600 / period_s,
        "duty_percent": 100 * on_s / period_s,
        "average_power_w": power_w * on_s / period_s,
        "min_temperature_c": min(c.min_c for c in cycles),
        "max_temperature_c": max(c.max_c for c in cycles),
    }


def analyze(settings: Settings, result: Simulation) -> dict:
    """Compare measured complete cycles against constant-ambient predictions."""
    cycles = full_cycles(result, settings.dt_s)
    # Ignore any cycle crossing the change instant: it contains two ambients.
    pre = [c for c in cycles if c.end_s <= settings.disturbance_at_s]
    post = [c for c in cycles if c.start_s >= settings.disturbance_at_s]
    measured_pre = average_cycles(pre, settings.power_w)
    measured_post = average_cycles(post, settings.power_w)

    # Evaluate the two model slopes at exactly the same T and heater state.
    idx = round(settings.disturbance_at_s / settings.dt_s)
    temp_at_change = float(result.temperature_c[idx])
    heater_at_change = int(result.heater_on[idx])
    slope_pre = (settings.power_w * heater_at_change - settings.heat_loss_w_per_k
                 * (temp_at_change - settings.ambient_before_c)
                 ) / settings.capacitance_j_per_k
    slope_post = (settings.power_w * heater_at_change - settings.heat_loss_w_per_k
                  * (temp_at_change - settings.ambient_after_c)
                  ) / settings.capacitance_j_per_k

    return {
        "settings": asdict(settings),
        "numerical_method": "Forward Euler; hysteresis evaluated each dt",
        "analytic_before": exact_cycle(settings, settings.ambient_before_c),
        "analytic_after": exact_cycle(settings, settings.ambient_after_c),
        "simulated_before": measured_pre,
        "simulated_after": measured_post,
        "transition": {
            "temperature_at_change_c": temp_at_change,
            "heater_state_at_change": heater_at_change,
            "hypothetical_rate_before_c_per_s": slope_pre,
            "rate_after_c_per_s": slope_post,
            "rate_change_c_per_s": slope_post - slope_pre,
            "first_entirely_post_disturbance_cycle_completed_after_s": (
                post[0].end_s - settings.disturbance_at_s
            ),
            "note": "Temperature is continuous. The controller enters a new periodic "
                    "cycle; there is no single steady temperature or PID-style settling time.",
        },
    }


def save_chart(settings: Settings, result: Simulation, output_path: Path) -> None:
    """Three aligned plots: process temperature, discrete heater, ambient step."""
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True,
                             gridspec_kw={"height_ratios": [3, 1.4, 1.4]})
    minutes = result.time_s / 60.0
    disturbance_min = settings.disturbance_at_s / 60.0

    axes[0].plot(minutes, result.temperature_c, label="Process temperature")
    axes[0].axhline(settings.heater_on_c, ls="--", label="Heater ON (38°C)")
    axes[0].axhline(settings.heater_off_c, ls="--", label="Heater OFF (42°C)")
    axes[0].set_ylabel("Process temperature (°C)")
    axes[0].legend(loc="best")

    axes[1].stairs(result.heater_on, minutes, baseline=None)
    axes[1].set_ylabel("Heater command")
    axes[1].set_yticks([0, 1], ["OFF", "ON"])
    axes[1].set_ylim(-0.15, 1.15)

    axes[2].stairs(result.ambient_c[:-1], minutes, baseline=None)
    axes[2].set_ylabel("Ambient (°C)")
    axes[2].set_xlabel("Time (minutes)")

    for ax in axes:
        ax.axvline(disturbance_min, color="black", ls=":", lw=1.3)
        ax.grid(alpha=0.25)
    axes[0].set_title("Thermal plant disturbance: ambient 25°C → 20°C")
    fig.tight_layout()
    fig.savefig(output_path, dpi=170)
    plt.close(fig)


def save_outputs(settings: Settings, result: Simulation, report: dict,
                 output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    # Temperature and ambient are states at time t[k]. Command is valid on
    # [t[k], t[k+1]); consequently the CSV omits the final state-only endpoint.
    rows = np.column_stack((result.time_s[:-1], result.temperature_c[:-1],
                            result.ambient_c[:-1], result.heater_on,
                            result.heating_rate_c_per_s))
    np.savetxt(output_dir / "timeseries.csv", rows, delimiter=",",
               header="time_s,temperature_c,ambient_c,heater_on,rate_c_per_s",
               comments="", fmt=["%.3f", "%.6f", "%.2f", "%d", "%.8f"])
    (output_dir / "summary.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8")
    save_chart(settings, result, output_dir / "disturbance_response.png")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dt", type=float, default=0.1, help="Euler timestep in seconds")
    parser.add_argument("--disturbance-at", type=float, default=1200.0,
                        help="Time of ambient step in seconds")
    parser.add_argument("--duration", type=float, default=3600.0,
                        help="Simulation duration in seconds")
    parser.add_argument("--before", type=float, default=25.0,
                        help="Ambient before the step (°C)")
    parser.add_argument("--after", type=float, default=20.0,
                        help="Ambient after the step (°C)")
    parser.add_argument("--output-dir", type=Path, default=Path("results"))
    args = parser.parse_args()
    settings = Settings(dt_s=args.dt, disturbance_at_s=args.disturbance_at,
                        total_time_s=args.duration,
                        ambient_before_c=args.before, ambient_after_c=args.after)
    result = simulate(settings)
    report = analyze(settings, result)
    save_outputs(settings, result, report, args.output_dir)

    print(f"Thermal time constant: {settings.tau_s:.1f} s")
    print(f"Disturbance: {settings.ambient_before_c:g} → {settings.ambient_after_c:g} °C "
          f"at t={settings.disturbance_at_s:g} s")
    print("\nMetric                         Before (sim / exact)    After (sim / exact)")
    for key, label, unit in [
        ("on_s", "Heater ON duration", "s"),
        ("off_s", "Heater OFF duration", "s"),
        ("period_s", "Cycle period", "s"),
        ("cycles_per_hour", "Cycles/hour", "/h"),
        ("duty_percent", "Duty cycle", "%"),
        ("average_power_w", "Average electrical power", "W"),
    ]:
        b, a = report["simulated_before"][key], report["simulated_after"][key]
        eb, ea = report["analytic_before"][key], report["analytic_after"][key]
        print(f"{label:<30} {b:8.2f} / {eb:8.2f} {unit:<2}   "
              f"{a:8.2f} / {ea:8.2f} {unit}")
    d = report["transition"]
    print(f"\nAt the disturbance T={d['temperature_at_change_c']:.3f} °C "
          "(no instantaneous T jump).")
    print(f"Slope change at fixed T and u: {d['rate_change_c_per_s']:+.3f} °C/s")
    print(f"Created {args.output_dir / 'disturbance_response.png'}")
    print(f"Created {args.output_dir / 'summary.json'}")
    print(f"Created {args.output_dir / 'timeseries.csv'}")


if __name__ == "__main__":
    main()
