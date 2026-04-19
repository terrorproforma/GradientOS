import { describe, expect, it } from "vitest";
import { mergeDriveFaultSnapshots, synthesizeDriveFaultSnapshotFromAxes } from "./App";

describe("synthesizeDriveFaultSnapshotFromAxes", () => {
  it("derives active drive power from live op-enabled axis data", () => {
    const snapshot = synthesizeDriveFaultSnapshotFromAxes({
      backend_name: "ethercat_rtcore",
      axis_absolute_feedback: [
        {
          axis: 0,
          logical_joint: 1,
          statusword: 0x27,
          error_code: 0,
          drive_native_truth_valid: true,
          drive_native_truth_reason: "valid",
          drive_native_truth_verification_source: "persisted_home_anchor_agreement",
          coordinate_system_valid: true,
        },
      ],
      servos: {
        "1": {
          logical_joint: 1,
          axis_index: 0,
          ds402_state: "OperationEnabled",
          statusword: 0x27,
          error_code: 0,
        },
      },
    });

    expect(snapshot).toMatchObject({
      servo_backend: "ethercat_rtcore",
      physical_state: "ACTIVE",
      driver_state: "ACTIVE",
      armed: 1,
      axis_enable_mask: 0x1,
      axis_enable_mask_hex: "0x1",
      enable_requested: true,
      requested_axes: 1,
      op_enabled_axes: 1,
      num_axes: 1,
      faulted_axes: 0,
      statusword_feedback_axes: 1,
    });
    expect(snapshot?.axes?.[0]).toMatchObject({
      axis: 0,
      logical_joint: 1,
      ds402_state: "OperationEnabled",
      statusword: 0x27,
      error_code: 0,
      native_home_active: false,
    });
  });

  it("derives a disarmed state from switch-on-disabled feedback", () => {
    const snapshot = synthesizeDriveFaultSnapshotFromAxes({
      backend_name: "ethercat_rtcore",
      axis_absolute_feedback: [
        {
          axis: 0,
          logical_joint: 1,
          statusword: 0x40,
          error_code: 0,
          drive_native_truth_valid: true,
          drive_native_truth_reason: "valid",
          drive_native_truth_verification_source: "persisted_home_anchor_agreement",
          coordinate_system_valid: true,
        },
      ],
      servos: {
        "1": {
          logical_joint: 1,
          axis_index: 0,
          ds402_state: "SwitchOnDisabled",
          statusword: 0x40,
          error_code: 0,
        },
      },
    });

    expect(snapshot).toMatchObject({
      servo_backend: "ethercat_rtcore",
      physical_state: "BUS_UP_DISARMED",
      driver_state: "DISARMED",
      armed: 0,
      axis_enable_mask: 0x0,
      axis_enable_mask_hex: "0x0",
      enable_requested: false,
      requested_axes: 0,
      op_enabled_axes: 0,
      num_axes: 1,
      faulted_axes: 0,
      statusword_feedback_axes: 1,
    });
  });
});

describe("mergeDriveFaultSnapshots", () => {
  it("overrides stale top-level power state with synthesized monitor data", () => {
    const previous = {
      servo_backend: "ethercat_rtcore",
      physical_state: "ACTIVE",
      driver_state: "ACTIVE",
      ethercat_master_state: "OP",
      rtcore_state: "UP",
      armed: 1,
      axis_enable_mask: 0x3f,
      axis_enable_mask_hex: "0x3f",
      enable_requested: true,
      requested_axes: 6,
      op_enabled_axes: 6,
      num_axes: 6,
      faulted_axes: 0,
      statusword_feedback_axes: 6,
      axes: [
        {
          axis: 0,
          logical_joint: 1,
          ds402_state: "OperationEnabled",
          statusword: 0x27,
          error_code: 0,
        },
      ],
    };
    const synthesized = synthesizeDriveFaultSnapshotFromAxes({
      backend_name: "ethercat_rtcore",
      axis_absolute_feedback: [
        {
          axis: 0,
          logical_joint: 1,
          statusword: 0x40,
          error_code: 0,
          drive_native_truth_valid: true,
          drive_native_truth_reason: "valid",
          drive_native_truth_verification_source: "persisted_home_anchor_agreement",
          coordinate_system_valid: true,
        },
      ],
      servos: {
        "1": {
          logical_joint: 1,
          axis_index: 0,
          ds402_state: "SwitchOnDisabled",
          statusword: 0x40,
          error_code: 0,
        },
      },
    });

    const merged = mergeDriveFaultSnapshots(previous, synthesized);

    expect(merged).toMatchObject({
      servo_backend: "ethercat_rtcore",
      physical_state: "BUS_UP_DISARMED",
      driver_state: "DISARMED",
      ethercat_master_state: "OP",
      rtcore_state: "UP",
      armed: 0,
      axis_enable_mask: 0x0,
      axis_enable_mask_hex: "0x0",
      enable_requested: false,
      requested_axes: 0,
      op_enabled_axes: 0,
      num_axes: 1,
      faulted_axes: 0,
      statusword_feedback_axes: 1,
    });
    expect(merged?.axes?.[0]).toMatchObject({
      axis: 0,
      logical_joint: 1,
      ds402_state: "SwitchOnDisabled",
      statusword: 0x40,
      error_code: 0,
    });
  });
});
