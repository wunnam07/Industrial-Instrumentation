"""Regression tests for separating thermal integration and controller sample time."""
import numpy as np
import pytest
from thermal_disturbance import Settings, simulate, full_cycles


def test_original_behavior_is_preserved_with_default_sample_time():
    a = simulate(Settings(total_time_s=200, disturbance_at_s=100))
    b = simulate(Settings(total_time_s=200, disturbance_at_s=100,
                          controller_sample_s=0.1))
    np.testing.assert_array_equal(a.temperature_c, b.temperature_c)
    np.testing.assert_array_equal(a.heater_on, b.heater_on)


def test_heater_command_can_change_only_at_controller_sample_instants():
    sample_steps = 100  # 10 s sample / 0.1 s Euler step.
    result = simulate(Settings(total_time_s=2400, disturbance_at_s=1200,
                               ambient_before_c=25, ambient_after_c=25,
                               controller_sample_s=10))
    changes = np.flatnonzero(result.heater_on[1:] != result.heater_on[:-1]) + 1
    assert changes.size > 0
    assert np.all(changes % sample_steps == 0)


def test_slower_sampling_widens_temperature_extrema_for_nominal_plant():
    params = dict(total_time_s=2400, disturbance_at_s=1200,
                  ambient_before_c=25, ambient_after_c=25)
    fast = full_cycles(simulate(Settings(**params, controller_sample_s=1)), 0.1)[-5:]
    slow = full_cycles(simulate(Settings(**params, controller_sample_s=10)), 0.1)[-5:]
    assert len(fast) == len(slow) == 5
    assert max(c.max_c for c in slow) > max(c.max_c for c in fast)
    assert min(c.min_c for c in slow) < min(c.min_c for c in fast)


@pytest.mark.parametrize("sample_s", [0, 0.05, 0.15, -1])
def test_nonintegral_or_nonpositive_controller_sampling_is_rejected(sample_s):
    with pytest.raises(ValueError, match="Controller sample time"):
        Settings(controller_sample_s=sample_s).validate()
