"""Run with: python -m pytest -q"""
import math

import pytest

from thermal_disturbance import Settings, analyze, exact_cycle, simulate


def test_parameters_are_physically_consistent():
    s = Settings()
    assert s.tau_s == pytest.approx(200)
    assert s.capacitance_j_per_k == pytest.approx(800)


def test_exact_analytical_cycle_predictions():
    s = Settings()
    warm = exact_cycle(s, 25)
    cold = exact_cycle(s, 20)
    assert warm["equilibrium_on_c"] == pytest.approx(50)
    assert cold["equilibrium_on_c"] == pytest.approx(45)
    assert warm["on_s"] == pytest.approx(81.09302, abs=0.001)
    assert cold["on_s"] == pytest.approx(169.45957, abs=0.001)
    assert warm["off_s"] == pytest.approx(53.657, abs=0.02)
    assert cold["off_s"] == pytest.approx(40.134, abs=0.02)
    assert cold["duty_percent"] > warm["duty_percent"]
    assert cold["cycles_per_hour"] < warm["cycles_per_hour"]


def test_first_euler_step_is_understood():
    s = Settings()
    result = simulate(s)
    # u=1 because T0=38: slope = (100 - 4*(38-25))/800 = 0.06 °C/s
    assert result.heater_on[0] == 1
    assert result.heating_rate_c_per_s[0] == pytest.approx(0.06)
    assert result.temperature_c[1] == pytest.approx(38.0 + 0.1 * 0.06)


def test_disturbance_changes_slope_without_a_temperature_jump():
    s = Settings()
    result = simulate(s)
    idx = round(s.disturbance_at_s / s.dt_s)
    assert result.ambient_c[idx - 1] == 25
    assert result.ambient_c[idx] == 20
    assert math.isfinite(result.temperature_c[idx])
    # Euler updated this state using the OLD ambient; no direct -5 °C reset.
    assert (abs(result.temperature_c[idx] - result.temperature_c[idx - 1])
            < 0.02)
    report = analyze(s, result)
    assert report["transition"]["rate_change_c_per_s"] == pytest.approx(-0.025)


def test_closed_loop_duty_and_period_match_exact_solutions():
    s = Settings()
    report = analyze(s, simulate(s))
    for zone in ("before", "after"):
        simulation = report[f"simulated_{zone}"]
        exact = report[f"analytic_{zone}"]
        assert simulation["number_of_cycles"] >= 5
        assert simulation["on_s"] == pytest.approx(exact["on_s"], abs=0.2)
        assert simulation["off_s"] == pytest.approx(exact["off_s"], abs=0.2)
        assert simulation["duty_percent"] == pytest.approx(exact["duty_percent"], abs=0.15)
        assert simulation["min_temperature_c"] >= 37.95
        assert simulation["max_temperature_c"] <= 42.05


def test_small_timestep_gives_similar_cycle_metrics():
    coarse = Settings(dt_s=0.1)
    fine = Settings(dt_s=0.01)
    a = analyze(coarse, simulate(coarse))
    b = analyze(fine, simulate(fine))
    for zone in ("before", "after"):
        assert a[f"simulated_{zone}"]["duty_percent"] == pytest.approx(
            b[f"simulated_{zone}"]["duty_percent"], abs=0.15)


def test_unreachable_upper_threshold_raises_error():
    s = Settings(power_w=60)
    with pytest.raises(ValueError, match="Cannot complete"):
        exact_cycle(s, 20)  # Teq=35°C; heater cannot reach 42°C
