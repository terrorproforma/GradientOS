from gradient_os import runtime_config


def test_load_runtime_config_defaults_to_manifest_policy(monkeypatch, tmp_path):
    cfg_path = tmp_path / "runtime.json"
    monkeypatch.setenv(runtime_config.RUNTIME_CONFIG_ENV_VAR, str(cfg_path))
    loaded = runtime_config.load_runtime_config()
    assert loaded["desired"]["robot"] == "gradient05"
    assert loaded["desired"]["active_tool_id"] is None
    assert loaded["desired"]["allow_unsafe_overrides"] is False
    assert loaded["desired"]["overrides"]["ik_solver_backend"] is None


def test_resolve_effective_runtime_uses_robot_policy_without_overrides():
    resolved = runtime_config.resolve_effective_runtime(
        robot_name="gradient05",
        sim_mode=False,
        requested_ik_solver_backend="ikfast",
        requested_servo_backend="simulation",
        allow_unsafe_overrides=False,
    )
    assert resolved["ik_solver"]["effective_backend"] == "numeric"
    assert resolved["ik_solver"]["source"] == "robot_policy"
    assert resolved["servo_backend"]["effective_backend"] == "ethercat_rtcore"
    assert resolved["servo_backend"]["source"] == "robot_policy"
    assert resolved["tool"]["active_tool_id"] == "identity"
    assert resolved["tool"]["source"] in {"library_default", "fallback_identity", "fallback_inline_identity"}


def test_resolve_effective_runtime_honors_dev_override_when_enabled():
    resolved = runtime_config.resolve_effective_runtime(
        robot_name="gradient05",
        sim_mode=False,
        requested_ik_solver_backend="ikfast",
        requested_servo_backend="feetech",
        allow_unsafe_overrides=True,
    )
    assert resolved["ik_solver"]["effective_backend"] == "ikfast"
    assert resolved["ik_solver"]["source"] == "dev_override"
    assert resolved["servo_backend"]["effective_backend"] == "feetech"
    assert resolved["servo_backend"]["source"] == "dev_override"


def test_compute_restart_required_from_active_vs_desired():
    active = runtime_config.resolve_effective_runtime(
        robot_name="gradient05",
        sim_mode=False,
        allow_unsafe_overrides=False,
    )
    desired = {
        "desired": {
            "robot": "gradient05",
            "allow_unsafe_overrides": False,
            "overrides": {
                "ik_solver_backend": None,
                "servo_backend": None,
            },
        }
    }
    assert runtime_config.compute_restart_required(active_runtime=active, desired_config=desired) is False

    desired["desired"]["robot"] = "gradient0"
    assert runtime_config.compute_restart_required(active_runtime=active, desired_config=desired) is True


def test_compute_restart_required_ignores_tool_mismatch_for_live_apply():
    active = runtime_config.resolve_effective_runtime(
        robot_name="gradient05",
        sim_mode=False,
        allow_unsafe_overrides=False,
        requested_active_tool_id="identity",
    )
    desired = {
        "desired": {
            "robot": "gradient05",
            "active_tool_id": "tig-torch-65deg",
            "allow_unsafe_overrides": False,
            "overrides": {
                "ik_solver_backend": None,
                "servo_backend": None,
            },
        }
    }
    assert runtime_config.compute_restart_required(active_runtime=active, desired_config=desired) is False

