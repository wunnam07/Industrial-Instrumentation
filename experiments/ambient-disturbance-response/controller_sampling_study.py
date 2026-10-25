"""Compare digital controller sampling times without changing the plant solver.

Run from this directory:
    python controller_sampling_study.py
All outputs describe an IDEAL, virtual plant. They are not physical validation.
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from thermal_disturbance import Settings, full_cycles, simulate

SAMPLE_INTERVALS_S = (0.1, 1.0, 5.0, 10.0, 20.0)


def run_study(dt_s: float = 0.1, duration_s: float = 6000.0):
    """Evaluate the last ten complete OFF-to-OFF cycles at constant ambient."""
    rows = []
    trajectories = {}
    for sample_s in SAMPLE_INTERVALS_S:
        settings = Settings(
            dt_s=dt_s,
            controller_sample_s=sample_s,
            ambient_before_c=25.0,
            ambient_after_c=25.0,  # Identical values: NO disturbance in this study.
            total_time_s=duration_s,
            disturbance_at_s=duration_s / 2,
            initial_temperature_c=38.0,
        )
        result = simulate(settings)
        cycles = full_cycles(result, dt_s)[-10:]
        if len(cycles) < 10:
            raise ValueError("Study duration is too short for ten full cycles.")
        total_cycle_time = sum(c.period_s for c in cycles)
        total_on_time = sum(c.on_s for c in cycles)
        rows.append({
            "controller_sample_s": sample_s,
            "plant_dt_s": dt_s,
            "min_temperature_c": min(c.min_c for c in cycles),
            "max_temperature_c": max(c.max_c for c in cycles),
            "mean_period_s": total_cycle_time / len(cycles),
            "cycles_per_hour": 3600 * len(cycles) / total_cycle_time,
            "duty_percent": 100 * total_on_time / total_cycle_time,
        })
        if sample_s in (1.0, 10.0):
            trajectories[sample_s] = result
    return rows, trajectories


def save_outputs(rows, trajectories, output_dir: Path):
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "controller_sampling_metrics.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    x = np.array([r["controller_sample_s"] for r in rows])
    mins = [r["min_temperature_c"] for r in rows]
    maxs = [r["max_temperature_c"] for r in rows]
    rates = [r["cycles_per_hour"] for r in rows]

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(x, mins, "o-", label="Minimum over last 10 cycles")
    axes[0].plot(x, maxs, "s-", label="Maximum over last 10 cycles")
    axes[0].axhline(38.0, color="gray", ls="--", label="38°C ON threshold")
    axes[0].axhline(42.0, color="black", ls="--", label="42°C OFF threshold")
    axes[0].set_ylabel("Temperature (°C)")
    axes[0].legend()
    axes[1].plot(x, rates, "o-")
    axes[1].axhline(30, color="black", ls=":", label="Illustrative 30 cycles/hour limit")
    axes[1].set_ylabel("Complete cycles per hour")
    axes[1].set_xlabel("Controller sampling interval (s)")
    axes[1].legend()
    for ax in axes:
        ax.set_xticks(x)
        ax.grid(alpha=0.25)
    fig.suptitle("Controller sampling study (Forward Euler dt = 0.1 s)")
    fig.tight_layout()
    fig.savefig(output_dir / "controller_sampling_metrics.png", dpi=170)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(11, 4.5))
    for sample_s, result in sorted(trajectories.items()):
        end = result.time_s[-1]
        keep = result.time_s >= (end - 600.0)
        ax.plot(result.time_s[keep] - (end - 600.0), result.temperature_c[keep],
                label=f"Controller samples every {sample_s:g} s")
    ax.axhline(38, color="gray", ls="--", label="ON threshold 38°C")
    ax.axhline(42, color="black", ls="--", label="OFF threshold 42°C")
    ax.set(xlabel="Time within final 600 s (s)", ylabel="Process temperature (°C)",
           title="Same physical plant; different controller sampling intervals")
    ax.legend()
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(output_dir / "controller_sampling_response.png", dpi=170)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("sampling_results"))
    args = parser.parse_args()
    rows, trajectories = run_study()
    save_outputs(rows, trajectories, args.output_dir)
    print("Sample(s)  Minimum(°C)  Maximum(°C)  Cycles/hour  Duty(%)")
    for r in rows:
        print(f'{r["controller_sample_s"]:8.1f}  {r["min_temperature_c"]:11.4f}'
              f'  {r["max_temperature_c"]:11.4f}  {r["cycles_per_hour"]:11.2f}'
              f'  {r["duty_percent"]:7.2f}')
    print(f"Plots and CSV saved in {args.output_dir}")


if __name__ == "__main__":
    main()
