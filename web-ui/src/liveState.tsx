import { createContext, useContext, type ReactNode } from "react";

export type Alert = {
  level: "error" | "warning" | "info";
  kind: string;
  message: string;
  servo_ids?: number[];
  ts?: number;
  details?: Record<string, unknown>;
};

export type ServoSample = {
  voltage_v?: number;
  temp_c?: number;
  current_a?: number;
  drive_duty_per_mille?: number;
  unloading_condition?: number;
  led_alarm_condition?: number;
  unloading_bits?: string;
  led_alarm_bits?: string;
  pos_counts?: number;
  torque_raw?: number;
  statusword?: number;
  statusword_hex?: string;
  error_code?: number;
  error_code_hex?: string;
  manufacturer_error_code?: number;
  manufacturer_error_code_hex?: string;
  startup_drive_config?: DriveStartupConfig | null;
  mode_display?: number;
  mode_display_name?: string;
  ds402_state?: string;
  ds402_state_code?: number;
  di_bits?: number;
  di_bits_hex?: string;
  axis_fault_flags?: number;
  brake_state?: number;
  logical_joint?: number | null;
  axis_index?: number;
  status_names?: string[];
};

export type DriveFaultReference = {
  profile_id?: string;
  label?: string;
  source_path?: string;
  source_path_relative?: string;
  available?: boolean;
};

export type DriveFaultDetail = {
  error_code_hex?: string;
  profile_id?: string;
  decoded?: boolean;
  code?: string;
  name?: string;
  class?: string | null;
  resettable?: boolean;
  bus_fault_code_hex?: string | null;
  bus_fault_name?: string | null;
  source?: string;
};

export type DriveStartupConfig = {
  profile_id?: string;
  setting_key?: string;
  setting_label?: string;
  object?: string;
  configured?: boolean;
  commanded?: number;
  commanded_value_label?: string;
  readback_valid?: boolean;
  readback?: number;
  readback_value_label?: string;
  verified?: boolean;
};

export type AbsoluteFeedbackField = {
  label?: string;
  valid?: boolean;
  value?: number;
};

export type DriveFaultAxis = {
  axis: number;
  logical_joint?: number | null;
  ds402_state: string;
  statusword: number;
  error_code: number;
  error_code_hex?: string;
  manufacturer_error_code?: number;
  manufacturer_error_code_hex?: string;
  startup_drive_config?: DriveStartupConfig | null;
  slave_online?: boolean;
  slave_operational?: boolean;
  slave_al_state?: number;
  slave_al_state_name?: string;
  pos_counts?: number;
  absolute_feedback?: Record<string, AbsoluteFeedbackField> | null;
  drive_native_truth_valid?: boolean;
  drive_native_truth_reason?: string;
  drive_native_truth_verification_source?: string;
  coordinate_system_valid?: boolean;
  native_home_state?: number;
  native_home_state_name?: string;
  native_home_active?: boolean;
  native_home_position_offset?: number;
  native_home_last_abort_code?: number;
  native_home_last_abort_code_hex?: string;
  fault?: DriveFaultDetail | null;
  manufacturer_fault?: DriveFaultDetail | null;
};

export type DriveFaultSnapshot = {
  servo_backend?: string | null;
  drive_profile?: string | null;
  configured_drive_profile?: string | null;
  live_drive_profile?: string | null;
  drive_profile_source?: string | null;
  fieldbus_profile?: string | null;
  reference?: DriveFaultReference | null;
  physical_state?: string;
  driver_state?: string;
  ethercat_master_state?: string;
  rtcore_state?: string;
  armed?: number;
  axis_enable_mask?: number;
  axis_enable_mask_hex?: string;
  native_home_active_axis_mask?: number;
  native_home_active_axis_mask_hex?: string;
  enable_requested?: boolean;
  requested_axes?: number;
  op_enabled_axes?: number;
  num_axes?: number;
  faulted_axes?: number;
  statusword_feedback_axes?: number;
  slave_online_axes?: number;
  slave_operational_axes?: number;
  link_up?: number;
  responding?: number;
  online?: number;
  operational?: number;
  startup_ready?: number;
  startup_drive_config_configured_axes?: number;
  startup_drive_config_verified_axes?: number;
  startup_drive_config_mismatch_axes?: number;
  wkc_actual?: number;
  wkc_expected?: number;
  master_al?: number;
  master_al_hex?: string;
  axes?: DriveFaultAxis[];
  metrics_path?: string;
};

export type MotionExecutionPayload = {
  controller_motion_state?: string;
  controller_thread_running?: boolean;
  state_name?: string;
  active_mode_name?: string;
  active_traj_id?: number;
  queue_depth?: number;
  queue_capacity?: number;
  motion_done?: boolean;
  stale_command?: boolean;
  underrun_count?: number;
  last_submitted_traj_id?: number;
};

export type ProgramStatusPayload = {
  name?: string | null;
  active?: boolean;
  state?: string;
  terminal_reason?: string | null;
  failing_step_index?: number | null;
  completed_step_count?: number;
  completed_loop_count?: number;
  loop_enabled?: boolean;
  use_cache?: boolean;
  step_count?: number;
  move_steps?: number;
  pause_steps?: number;
  joint_move_steps?: number;
  rtcore_segments?: boolean;
  segment_execution_policy?: string;
  current_step_index?: number | null;
  current_step_type?: string | null;
  loop_iteration?: number;
};

export type MotionStatusResponse = {
  status?: string;
  detail?: string;
  accepted?: boolean;
  command_acknowledged?: boolean;
  state?: string;
  completion_scope?: string;
  trajectory_id?: number;
  source_of_truth?: string;
  execution?: MotionExecutionPayload;
  program?: ProgramStatusPayload;
};

export type MonitorCommsSummary = {
  command_link_stale?: boolean;
  last_command_age_s?: number;
  recent_udp_reset_count?: number;
  last_command_peer?: string;
};

export type LiveTelemetryEvent = {
  timestamp: number;
  raw: string;
  joints?: number[];
  display_joints?: number[];
  joint_feedback_available?: boolean;
  joint_feedback_stale?: boolean;
  joint_feedback_stale_age_s?: number;
  joint_feedback_error?: string;
  gripper?: number;
  servos?: Record<string, ServoSample>;
  alerts?: Alert[];
  drive_faults?: DriveFaultSnapshot | null;
  weld_active?: boolean;
  weld_type?: string;
  comms?: MonitorCommsSummary;
  motion_status?: MotionStatusResponse | null;
};

export type LiveStateContextValue = {
  apiHost: string;
  latest: LiveTelemetryEvent | null;
  motionStatus: MotionStatusResponse | null;
  setMotionStatus: (next: MotionStatusResponse | null) => void;
  isConnected: boolean;
  monitorLastMessageAtMs: number | null;
  monitorFreshnessMs: number | null;
  isMonitorFresh: boolean;
  monitorError: string | null;
  apiHealthError: string | null;
};

export const LIVE_MONITOR_STALE_MS = 250;
export const STANDALONE_JOINT_FEEDBACK_POLL_MS = 100;

const LiveStateContext = createContext<LiveStateContextValue | null>(null);

export function LiveStateProvider({
  value,
  children,
}: {
  value: LiveStateContextValue;
  children: ReactNode;
}) {
  return <LiveStateContext.Provider value={value}>{children}</LiveStateContext.Provider>;
}

export function useOptionalLiveState(): LiveStateContextValue | null {
  return useContext(LiveStateContext);
}

export function useLiveState(): LiveStateContextValue {
  const value = useOptionalLiveState();
  if (!value) {
    throw new Error("useLiveState must be used within a LiveStateProvider.");
  }
  return value;
}
