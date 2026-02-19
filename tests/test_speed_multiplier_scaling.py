import math

import pytest

from gradient_os.arm_controller import command_api
from gradient_os.arm_controller import utils


def _set_profile_defaults(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(utils, "DEFAULT_PROFILE_VELOCITY", 0.1, raising=False)
    monkeypatch.setattr(utils, "DEFAULT_PROFILE_ACCELERATION", 0.05, raising=False)


def test_resolve_profile_params_default_multiplier(monkeypatch: pytest.MonkeyPatch) -> None:
    _set_profile_defaults(monkeypatch)

    multiplier, velocity, acceleration = command_api._resolve_profile_params_for_speed_multiplier(1.0)

    assert multiplier == pytest.approx(1.0)
    assert velocity == pytest.approx(0.1)
    assert acceleration == pytest.approx(0.05)


def test_resolve_profile_params_scales_velocity_and_acceleration(monkeypatch: pytest.MonkeyPatch) -> None:
    _set_profile_defaults(monkeypatch)

    multiplier, velocity, acceleration = command_api._resolve_profile_params_for_speed_multiplier(3.0)

    assert multiplier == pytest.approx(3.0)
    assert velocity == pytest.approx(0.3)
    # Quadratic scaling keeps short acceleration-limited moves responsive to multiplier changes.
    assert acceleration == pytest.approx(0.45)


def test_resolve_profile_params_clamps_and_handles_invalid(monkeypatch: pytest.MonkeyPatch) -> None:
    _set_profile_defaults(monkeypatch)

    invalid_multiplier, invalid_v, invalid_a = command_api._resolve_profile_params_for_speed_multiplier("oops")
    assert invalid_multiplier == pytest.approx(1.0)
    assert invalid_v == pytest.approx(0.1)
    assert invalid_a == pytest.approx(0.05)

    nan_multiplier, _, _ = command_api._resolve_profile_params_for_speed_multiplier(float("nan"))
    inf_multiplier, _, _ = command_api._resolve_profile_params_for_speed_multiplier(float("inf"))
    low_multiplier, _, _ = command_api._resolve_profile_params_for_speed_multiplier(0.0)
    high_multiplier, _, _ = command_api._resolve_profile_params_for_speed_multiplier(100.0)

    assert math.isfinite(nan_multiplier)
    assert math.isfinite(inf_multiplier)
    assert nan_multiplier == pytest.approx(1.0)
    assert inf_multiplier == pytest.approx(1.0)
    assert low_multiplier == pytest.approx(0.1)
    assert high_multiplier == pytest.approx(10.0)
