import { Activity, AlertTriangle, Power, ShieldAlert, ShieldCheck } from "lucide-react";
import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { DEFAULT_SPEED_SLIDER } from "./uiConstants";
import {
	STANDALONE_JOINT_FEEDBACK_POLL_MS,
	useOptionalLiveState,
} from "./liveState";

type Props = {
	apiHost: string;
	driveFaults?: DriveFaultSnapshot | null;
	activeServoBackend?: string | null;
	onJointFeedback?: (anglesDeg: number[], gripperDeg?: number) => void;
	onError?: (message: string) => void;
	motionStatus?: MotionStatusResponse | null;
	onMotionStatus?: (status: MotionStatusResponse | null) => void;
	splitStatusSections?: boolean;
	showStatusSections?: boolean;
	showGripperPanel?: boolean;
	showSoftwareZeroButton?: boolean;
	controlsCollapsed?: boolean;
	onToggleControlsCollapsed?: () => void;
};

type RuntimeHeaderProps = {
	apiHost: string;
	driveFaults?: DriveFaultSnapshot | null;
	activeServoBackend?: string | null;
	onJointFeedback?: (anglesDeg: number[], gripperDeg?: number) => void;
	onError?: (message: string) => void;
	motionStatus?: MotionStatusResponse | null;
	onMotionStatus?: (status: MotionStatusResponse | null) => void;
	className?: string;
};

type JointInfoValue = number | null;

type JointInfoResponse = {
	arm_deg?: JointInfoValue[];
	arm_rad?: JointInfoValue[];
	arm_display_deg?: JointInfoValue[];
	arm_display_rad?: JointInfoValue[];
	gripper_deg?: number;
	gripper_rad?: number;
	read_source?: string;
	raw_canonical_joint_truth_available?: boolean;
	display_joint_truth_available?: boolean;
	canonical_joint_truth_available?: boolean;
};

function normalizeJointAngles(values: JointInfoValue[] | null | undefined): number[] | null {
	if (Array.isArray(values) && values.length > 0) {
		return values.map((value) => (typeof value === "number" && Number.isFinite(value) ? Number(value) : Number.NaN));
	}
	return null;
}

function mergeDisplayWithCanonicalFallback(
	primary: number[] | null,
	fallback: number[] | null,
): number[] | null {
	// Per-joint display/canonical merge, scoped deliberately narrow:
	//   * Display snapshot fully missing -> return null. The commissioning
	//     panel has always treated "no operator display at all" as a
	//     distinct state; we do NOT substitute canonical wholesale because
	//     that would leak cached/canonical values into the visualizer as
	//     if they were live operator display.
	//   * Display snapshot partially present (e.g. five joints finite and
	//     one NaN after a full-shaft-turn excursion makes the per-joint
	//     display-mode command-roundtrip gate fail) -> fill the NaN slots
	//     from canonical. This preserves the operator's ability to SEE
	//     the live angle of the affected joint (otherwise the panel flips
	//     to "--" and the Drive Home button becomes unreachable while the
	//     fallback poller flickers the state) without disturbing joints
	//     whose display truth IS live.
	// The caller is responsible for only passing a canonical `fallback`
	// array when canonical truth is explicitly authoritative (see the
	// call sites in preferredJointAnglesDeg / preferredTelemetryJointAnglesRad).
	if (!primary) {
		return null;
	}
	if (!fallback) {
		return primary;
	}
	const length = Math.max(primary.length, fallback.length);
	const merged: number[] = [];
	for (let index = 0; index < length; index += 1) {
		const primaryValue = primary[index];
		const fallbackValue = fallback[index];
		if (Number.isFinite(primaryValue)) {
			merged.push(Number(primaryValue));
		} else if (Number.isFinite(fallbackValue)) {
			merged.push(Number(fallbackValue));
		} else {
			merged.push(Number.NaN);
		}
	}
	return merged;
}

function preferredJointAnglesDeg(payload: JointInfoResponse | null | undefined): number[] | null {
	const display = normalizeJointAngles(payload?.arm_display_deg);
	// Only fall back to canonical `arm_deg` when the backend tells us raw
	// canonical truth is live. If canonical itself is cached/unavailable
	// (e.g. read_source="unavailable"), arm_deg can be a stale setpoint
	// and we must NOT leak it into the commissioning panel.
	const canonicalAuthoritative = Boolean(
		payload?.raw_canonical_joint_truth_available
			?? payload?.canonical_joint_truth_available
			?? (payload?.read_source === "live_feedback"),
	);
	const canonical = canonicalAuthoritative ? normalizeJointAngles(payload?.arm_deg) : null;
	return mergeDisplayWithCanonicalFallback(display, canonical);
}

export function preferredTelemetryJointAnglesRad(
	telemetry: {
		display_joints?: JointInfoValue[];
		joints?: JointInfoValue[];
		raw_canonical_joint_truth_available?: boolean;
		canonical_joint_truth_available?: boolean;
		joint_feedback_stale?: boolean;
	} | null | undefined,
): number[] | null {
	const display = Array.isArray(telemetry?.display_joints) && telemetry.display_joints.length > 0
		? normalizeJointAngles(telemetry.display_joints)
		: null;
	// Same rule as preferredJointAnglesDeg: only treat `joints` as a
	// per-joint fallback when canonical truth is explicitly live. On the
	// SSE monitor stream the field may be missing; if it is missing AND
	// we have finite `joints` values AND `display_joints` has nothing,
	// we still prefer to surface the canonical values (the operator
	// needs to see something) so allow a missing flag to count as
	// "available" when it is accompanied by a finite joints array.
	const canonicalExplicitlyAvailable = Boolean(
		telemetry?.raw_canonical_joint_truth_available
			?? telemetry?.canonical_joint_truth_available,
	);
	const hasCanonicalArray = Array.isArray(telemetry?.joints) && telemetry.joints.length > 0;
	const canonicalAuthoritative = canonicalExplicitlyAvailable
		|| Boolean(telemetry?.joint_feedback_stale)
		|| (telemetry?.raw_canonical_joint_truth_available === undefined
			&& telemetry?.canonical_joint_truth_available === undefined
			&& hasCanonicalArray);
	const canonical = canonicalAuthoritative && hasCanonicalArray
		? normalizeJointAngles(telemetry?.joints)
		: null;
	if (!display && canonical) {
		return canonical;
	}
	return mergeDisplayWithCanonicalFallback(display, canonical);
}

function hasAnyFiniteJointAngles(values: number[] | null | undefined): values is number[] {
	return Array.isArray(values) && values.some((value) => Number.isFinite(value));
}

function hasAllFiniteJointAngles(values: number[] | null | undefined): values is number[] {
	return Array.isArray(values) && values.length > 0 && values.every((value) => Number.isFinite(value));
}

function jointAngleArraysMatch(current: number[], next: number[]): boolean {
	if (current.length !== next.length) {
		return false;
	}
	for (let index = 0; index < next.length; index += 1) {
		const currentValue = current[index];
		const nextValue = next[index];
		const currentFinite = Number.isFinite(currentValue);
		const nextFinite = Number.isFinite(nextValue);
		if (currentFinite !== nextFinite) {
			return false;
		}
		if (currentFinite && Math.abs(currentValue - nextValue) > 1e-3) {
			return false;
		}
	}
	return true;
}

const POWER_TRANSITION_SAFE_STABLE_MS = 600;

type MotionExecutionPayload = {
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
	safe_for_power_transition?: boolean;
	power_transition_blockers?: string[];
	power_transition_blocker_details?: PowerTransitionBlocker[];
	power_transition_feedback_synchronized?: boolean;
	power_transition_faulted_axis_count?: number;
	power_transition_active_jog?: boolean;
};

type PowerTransitionBlocker = {
	code?: string;
	message?: string;
	active_traj_id?: number;
	queue_depth?: number;
	active_jog_axis_mask?: number;
	faulted_axis_count?: number;
	faulted_axis_indices?: number[];
	reason?: string;
	truth_reasons?: string[];
	truth_unavailable_axes?: number[];
	truth_unavailable_joints?: number[];
	statuswords?: string[];
	requires_native_home?: boolean;
};

type MotionStatusResponse = {
	status?: string;
	accepted?: boolean;
	command_acknowledged?: boolean;
	state?: string;
	completion_scope?: string;
	trajectory_id?: number;
	source_of_truth?: string;
	execution?: MotionExecutionPayload;
	safe_for_power_transition?: boolean;
	power_transition_blockers?: string[];
	power_transition_blocker_details?: PowerTransitionBlocker[];
	power_action?: string;
	code?: string;
	message?: string;
	wait_for_idle_requested?: boolean;
	waited_for_idle?: boolean;
	baseline_source?: string;
	disarmed_after_reset?: boolean;
	backend_handled?: boolean;
};

type PowerTransitionStatusView = {
	safe_for_power_transition?: boolean;
	power_transition_blocker_details?: PowerTransitionBlocker[];
};

type JogCommandVector = [number, number, number, number, number, number];
type JogStatePayload = {
	active: boolean;
	deadman: boolean;
	vx: number;
	vy: number;
	vz: number;
	v_roll: number;
	v_pitch: number;
	v_yaw: number;
	stop_reason?: string;
};

type JogSessionSnapshot = {
	session_id?: string | null;
	state?: string;
	last_seq_received?: number;
	last_seq_applied?: number;
	lease_remaining_s?: number | null;
	last_stop_reason?: string | null;
};

type JogSessionResponse = {
	status?: string;
	session?: JogSessionSnapshot;
};

type CommissioningStatus = {
	tone: "info" | "success" | "warning" | "error";
	message: string;
};

type CommissioningMessageTone = CommissioningStatus["tone"] | "neutral";

type CommissioningMessageEntry = {
	key: string;
	tone: CommissioningMessageTone;
	message: string;
};

type NativeHomeResponse = {
	status?: string;
	accepted?: boolean;
	verified?: boolean;
	timed_out?: boolean;
	code?: string;
	message?: string;
	joint?: number;
	detail?: string;
	native_home_state?: number;
	native_home_state_name?: string;
};

type DriveFaultDetail = {
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

type DriveStartupConfig = {
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

type AbsoluteFeedbackField = {
	label?: string;
	valid?: boolean;
	value?: number;
};

type DriveFaultAxis = {
	axis: number;
	logical_joint?: number | null;
	ds402_state: string;
	statusword: number;
	error_code: number;
	error_code_hex?: string;
	manufacturer_error_code?: number;
	manufacturer_error_code_hex?: string;
	startup_drive_config?: DriveStartupConfig | null;
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
	native_home_state_reported?: number;
	native_home_state_reported_name?: string;
	native_home_last_abort_code_reported?: number;
	native_home_last_abort_code_reported_hex?: string;
	native_home_verification_source?: string;
	fault?: DriveFaultDetail | null;
	manufacturer_fault?: DriveFaultDetail | null;
	// Phase 2 (2026-04-20): extended A6-EC 0x2040 PDO diagnostics.
	// Fields are optional and null-safe: if the drive rejected the
	// extended PDO mapping, none of them are set, so absence means
	// "not present" rather than "zero-valid".
	bus_voltage_v?: number | null;
	load_rate_pct?: number | null;
	igbt_temp_c?: number | null;
	motor_temp_c?: number | null;
	position_error_counts?: number | null;
	drive_not_ready_bits?: number | null;
	drive_not_ready_text?: string | null;
	motor_not_rotating_code?: number | null;
	motor_not_rotating_text?: string | null;
};

type DriveFaultReference = {
	profile_id?: string;
	label?: string;
	available?: boolean;
};

type DriveFaultSnapshot = {
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
	startup_drive_config_configured_axes?: number;
	startup_drive_config_verified_axes?: number;
	startup_drive_config_mismatch_axes?: number;
	axes?: DriveFaultAxis[];
};

const JOINT_STEP_OPTIONS_DEG = [0.25, 1, 5] as const;
const COMMISSIONING_MESSAGE_SLOT_COUNT = 3;

function commissioningMessageToneClasses(tone: CommissioningMessageTone): string {
	if (tone === "success") {
		return "border-emerald-500/30 bg-emerald-400/10 text-emerald-100";
	}
	if (tone === "warning") {
		return "border-amber-500/30 bg-amber-400/10 text-amber-100";
	}
	if (tone === "error") {
		return "border-rose-500/30 bg-rose-400/10 text-rose-100";
	}
	if (tone === "info") {
		return "border-cyan-500/20 bg-cyan-400/10 text-cyan-100";
	}
	return "border-slate-700/60 bg-slate-900/70 text-slate-400";
}
const MIN_CUSTOM_JOINT_STEP_DEG = 0.001;
const JOG_SESSION_LEASE_TIMEOUT_S = 1.0;

function createJogOwnerId(): string {
	try {
		if (typeof crypto !== "undefined" && typeof crypto.randomUUID === "function") {
			return crypto.randomUUID();
		}
	} catch {
		// Fall through to timestamp-based fallback.
	}
	return `web-ui-${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 10)}`;
}

function getJogSessionErrorCode(message: string | null | undefined): string | null {
	if (!message) {
		return null;
	}
	const match = message.match(/\b(SESSION_EXPIRED|OWNER_CONFLICT|WRONG_SESSION|SESSION_NOT_FOUND|SESSION_INACTIVE)\b/);
	return match ? match[1] : null;
}

function isJogSessionTerminalError(message: string | null | undefined): boolean {
	return getJogSessionErrorCode(message) !== null;
}

function isJogSessionRecoverableError(code: string | null): boolean {
	return code === "SESSION_EXPIRED" || code === "SESSION_NOT_FOUND" || code === "SESSION_INACTIVE";
}

function formatDriveFaultAxisLabel(axis: DriveFaultAxis): string {
	if (typeof axis.logical_joint === "number" && Number.isFinite(axis.logical_joint)) {
		return `J${axis.logical_joint} / axis${axis.axis}`;
	}
	return `axis${axis.axis}`;
}

function formatDriveFaultDescription(axis: DriveFaultAxis): string {
	const parts = [
		`${formatDriveFaultAxisLabel(axis)}: ${axis.ds402_state}`,
		`err=0x${axis.error_code.toString(16).padStart(4, "0")}`,
	];
	const busCode = axis.fault?.code?.trim();
	const busName = axis.fault?.name?.trim();
	if (busCode) {
		parts.push(busCode);
	}
	if (busName) {
		parts.push(busName);
	}
	if (typeof axis.manufacturer_error_code === "number" && axis.manufacturer_error_code !== 0) {
		parts.push(`mfg=0x${axis.manufacturer_error_code.toString(16).padStart(8, "0")}`);
	}
	const manufacturerCode = axis.manufacturer_fault?.code?.trim();
	const manufacturerName = axis.manufacturer_fault?.name?.trim();
	if (manufacturerCode) {
		parts.push(manufacturerCode);
	}
	if (manufacturerName) {
		parts.push(manufacturerName);
	}
	if (axis.fault?.resettable === true) {
		parts.push("resettable");
	} else if (axis.manufacturer_fault?.resettable === true) {
		parts.push("resettable");
	}
	const startupConfig = axis.startup_drive_config;
	if (startupConfig?.configured) {
		const label = startupConfig.setting_label?.trim() || startupConfig.setting_key?.trim() || "startup config";
		const commanded = startupConfig.commanded ?? 0;
		const commandedLabel = startupConfig.commanded_value_label?.trim() || String(commanded);
		if (startupConfig.readback_valid) {
			const readback = startupConfig.readback ?? 0;
			const readbackLabel = startupConfig.readback_value_label?.trim() || String(readback);
			parts.push(
				startupConfig.verified
					? `${label}: ${commandedLabel} verified`
					: `${label}: expected ${commandedLabel}, read ${readbackLabel}`,
			);
		} else {
			parts.push(`${label}: ${commandedLabel} unread`);
		}
	}
	return parts.join(" | ");
}

function formatNativeHomeStatus(
	axis: DriveFaultAxis | undefined,
	driveFaults: DriveFaultSnapshot | null | undefined,
): string | null {
	if (!axis) {
		return null;
	}
	const activeMask = driveFaults?.native_home_active_axis_mask ?? 0;
	const activeForAxis = Boolean(axis.native_home_active) || ((activeMask & (1 << axis.axis)) !== 0);
	if (activeForAxis) {
		return "Drive Home requested...";
	}
	const stateName = (axis.native_home_state_name ?? "").trim().toLowerCase();
	if (!stateName || stateName === "idle") {
		return null;
	}
	if (stateName === "requested") {
		return "Drive Home requested...";
	}
	const axisMask = driveFaults?.axis_enable_mask ?? 0;
	const axisCurrentlyDisarmed = (axisMask & (1 << axis.axis)) === 0;
	if (stateName === "succeeded") {
		const parts = ["Drive Home succeeded"];
		if (typeof axis.native_home_position_offset === "number") {
			parts.push(`offset ${axis.native_home_position_offset} cnt`);
		}
		if (axisCurrentlyDisarmed) {
			parts.push("axis currently disarmed");
		}
		return parts.join(" | ");
	}
	if (stateName === "failed") {
		const abortCode = axis.native_home_last_abort_code_hex ?? "0x00000000";
		return `Drive Home failed | abort ${abortCode}`;
	}
	return `Drive Home ${stateName}`;
}

type DriveNativeTruthStatusView = {
	message: string;
	tone: "valid" | "warning" | "invalid";
};

function isHardTruthBlocker(reason: string | undefined): boolean {
	return [
		"raw_feedback_missing",
		"logical_joint_unmapped",
		"drive_native_fault_present",
		"drive_native_manufacturer_fault_present",
		"drive_native_slave_offline",
		"drive_native_slave_not_operational",
		"drive_native_absolute_home_anchor_missing",
		"multi_turn_feedback_lost_across_power_cycle",
		"multi_turn_feedback_invalid",
		"encoder_retention_fault_present",
	].includes(String(reason ?? ""));
}

function formatDriveNativeTruthStatus(axis: DriveFaultAxis | undefined): DriveNativeTruthStatusView | null {
	if (!axis) {
		return null;
	}
	const verificationSource = (axis.drive_native_truth_verification_source ?? "").trim();
	const truthReason = (axis.drive_native_truth_reason ?? "").trim();
	if (axis.drive_native_truth_valid === true) {
		if (verificationSource.length > 0) {
			return {
				message: `Canonical truth trust: ${verificationSource}`,
				tone: "valid",
			};
		}
		if (axis.coordinate_system_valid === true) {
			return {
				message: "Canonical truth trust: verified",
				tone: "valid",
			};
		}
		return null;
	}
	if (truthReason.length > 0) {
		if (!isHardTruthBlocker(truthReason)) {
			return {
				message: `Canonical truth trust warning: ${truthReason}`,
				tone: "warning",
			};
		}
		return {
			message: `Canonical truth unavailable: ${truthReason}`,
			tone: "invalid",
		};
	}
	return null;
}

// Phase 2 (2026-04-20): per-axis health chips rendered under the
// canonical-truth trust line. Chips are colour-coded so the operator
// can scan the panel at a glance and spot a hot IGBT or a high-load
// joint without reading raw numbers. Thresholds here are operator-
// facing warning bands, NOT control-loop limits; actual drive
// protection still lives in the firmware's Er codes.
type AxisHealthChipTone = "ok" | "warn" | "error" | "info";

type AxisHealthChip = {
	key: string;
	label: string;
	tone: AxisHealthChipTone;
};

const AXIS_HEALTH_CHIP_TONE_CLASSES: Record<AxisHealthChipTone, string> = {
	ok: "bg-emerald-900/60 text-emerald-200 border border-emerald-500/30",
	warn: "bg-amber-900/60 text-amber-200 border border-amber-500/30",
	error: "bg-rose-900/60 text-rose-100 border border-rose-500/40",
	info: "bg-slate-800/80 text-slate-200 border border-slate-600/40",
};
const JOINT_STEP_DEBOUNCE_MS = 125;

function buildAxisHealthChips(axis: DriveFaultAxis | undefined): AxisHealthChip[] {
	if (!axis) {
		return [];
	}
	const chips: AxisHealthChip[] = [];
	if (typeof axis.bus_voltage_v === "number" && Number.isFinite(axis.bus_voltage_v)) {
		const v = axis.bus_voltage_v;
		const tone: AxisHealthChipTone = v < 18 || v > 60 ? "warn" : "ok";
		chips.push({ key: "vbus", label: `${v.toFixed(1)} V`, tone });
	}
	if (typeof axis.igbt_temp_c === "number" && Number.isFinite(axis.igbt_temp_c)) {
		const t = axis.igbt_temp_c;
		const tone: AxisHealthChipTone = t > 85 ? "error" : t > 70 ? "warn" : "ok";
		chips.push({ key: "igbt", label: `IGBT ${t}°C`, tone });
	}
	if (typeof axis.motor_temp_c === "number" && Number.isFinite(axis.motor_temp_c)) {
		const t = axis.motor_temp_c;
		const tone: AxisHealthChipTone = t > 100 ? "error" : t > 70 ? "warn" : "ok";
		chips.push({ key: "motor", label: `Motor ${t}°C`, tone });
	}
	if (typeof axis.load_rate_pct === "number" && Number.isFinite(axis.load_rate_pct)) {
		const l = axis.load_rate_pct;
		const tone: AxisHealthChipTone = l > 80 ? "warn" : "ok";
		chips.push({ key: "load", label: `Load ${l.toFixed(0)}%`, tone });
	}
	if (
		typeof axis.position_error_counts === "number" &&
		Number.isFinite(axis.position_error_counts) &&
		Math.abs(axis.position_error_counts) > 100
	) {
		chips.push({
			key: "pe",
			label: `PE ${axis.position_error_counts}`,
			tone: "warn",
		});
	}
	if (
		typeof axis.motor_not_rotating_text === "string" &&
		axis.motor_not_rotating_text.trim().length > 0 &&
		axis.motor_not_rotating_text !== "ok"
	) {
		chips.push({
			key: "mnr",
			label: axis.motor_not_rotating_text,
			tone: "info",
		});
	}
	if (
		typeof axis.drive_not_ready_text === "string" &&
		axis.drive_not_ready_text.trim().length > 0 &&
		axis.drive_not_ready_text !== "ready"
	) {
		chips.push({
			key: "dnr",
			label: axis.drive_not_ready_text,
			tone: "warn",
		});
	}
	return chips;
}

function expSliderToMultiplier(v: number): number {
	// v in [0..1000] → 10^((t*2)-1) with t=v/1000
	const t = Math.max(0, Math.min(1000, v)) / 1000;
	const expVal = (t * 2) - 1;
	let mult = Math.pow(10, expVal);
	if (mult < 0.1) mult = 0.1;
	if (mult > 10) mult = 10;
	return mult;
}

function isZeroJogPayload(payload: JogStatePayload): boolean {
	return payload.vx === 0 &&
		payload.vy === 0 &&
		payload.vz === 0 &&
		payload.v_roll === 0 &&
		payload.v_pitch === 0 &&
		payload.v_yaw === 0;
}

function formatStepDegrees(value: number): string {
	if (!Number.isFinite(value)) {
		return "--";
	}
	return Number(value.toFixed(3)).toString();
}

function formatMotionStateLabel(value: string | undefined): string {
	const raw = (value ?? "").trim();
	if (!raw) {
		return "UNKNOWN";
	}
	return raw.replace(/_/g, " ").toUpperCase();
}

function formatPowerTransitionBlocker(blocker: PowerTransitionBlocker): string {
	const code = (blocker.code ?? "").trim().toLowerCase();
	if (code === "active_trajectory") {
		return `Active trajectory ${blocker.active_traj_id ?? "unknown"} is still latched.`;
	}
	if (code === "queued_motion") {
		return `Queued RTCore motion remains pending (${blocker.queue_depth ?? 0} points).`;
	}
	if (code === "active_jog") {
		return "A jog command is still active.";
	}
	if (code === "stale_command") {
		return "RTCore reports a stale command and must be resynchronized.";
	}
	if (code === "fault_present") {
		return `One or more drive faults are still present (${blocker.faulted_axis_count ?? 0}).`;
	}
	if (code === "coordinate_system_invalid") {
		const joints = Array.isArray(blocker.truth_unavailable_joints)
			? blocker.truth_unavailable_joints.filter((value) => Number.isFinite(value)).map((value) => Number(value))
			: [];
		const statuswords = Array.isArray(blocker.statuswords)
			? blocker.statuswords.map((value) => String(value).trim()).filter((value) => value.length > 0)
			: [];
		const jointSuffix = joints.length > 0
			? ` on joint${joints.length === 1 ? "" : "s"} ${joints.join(", ")}`
			: "";
		const statusSuffix = statuswords.length > 0
			? ` (status ${statuswords.join(", ")})`
			: "";
		return `Drive coordinate system is invalid${jointSuffix}${statusSuffix}; run Drive Home before power-up.`;
	}
	if (code === "canonical_truth_unavailable") {
		return "Live joint truth is unavailable; keep the drives disarmed until telemetry is valid.";
	}
	if (code === "not_synchronized") {
		return "Live feedback has not been synchronized to a safe hold target yet.";
	}
	if (code === "controller_thread_running") {
		return "A controller motion/program thread is still running.";
	}
	if (code === "motion_active") {
		return "RTCore still reports active motion execution.";
	}
	return blocker.message?.trim() || "Power transition is currently blocked.";
}

async function readErrorMessage(res: Response): Promise<string> {
	const contentType = res.headers.get("content-type") ?? "";
	if (contentType.includes("application/json")) {
		try {
			const payload = await res.json() as {
				detail?: unknown;
				message?: unknown;
			};
			if (payload && typeof payload === "object") {
				if (
					payload.detail &&
					typeof payload.detail === "object" &&
					typeof (payload.detail as { message?: unknown }).message === "string"
				) {
					const detailPayload = payload.detail as { code?: unknown; message: string };
					if (typeof detailPayload.code === "string" && detailPayload.code.trim()) {
						return `${detailPayload.code}: ${detailPayload.message}`;
					}
					return String(detailPayload.message);
				}
				if (typeof payload.detail === "string") {
					return payload.detail;
				}
				if (
					payload.detail &&
					typeof payload.detail === "object" &&
					typeof (payload.detail as { code?: unknown }).code === "string"
				) {
					return String((payload.detail as { code: string }).code);
				}
				if (typeof payload.message === "string") {
					return payload.message;
				}
				return JSON.stringify(payload);
			}
		} catch {
			// Fall through to plain text handling.
		}
	}
	try {
		const text = (await res.text()).trim();
		if (text) {
			return text;
		}
	} catch {
		// Ignore read failures and use the HTTP status line.
	}
	return `${res.status} ${res.statusText}`;
}

export function ControlPanelRuntimeHeader({
	apiHost,
	driveFaults,
	activeServoBackend,
	onJointFeedback,
	onError,
	motionStatus: controlledMotionStatus,
	onMotionStatus,
	className,
}: RuntimeHeaderProps) {
	const liveState = useOptionalLiveState();
	const sharedMotionStatus = liveState?.motionStatus ?? null;
	const setSharedMotionStatus = liveState?.setMotionStatus;
	const [localMotionStatus, setLocalMotionStatus] = useState<MotionStatusResponse | null>(null);
	const motionStatus = controlledMotionStatus ?? sharedMotionStatus ?? localMotionStatus;
	const setMotionStatus = useCallback((next: MotionStatusResponse | null) => {
		if (controlledMotionStatus === undefined && !setSharedMotionStatus) {
			setLocalMotionStatus(next);
		}
		setSharedMotionStatus?.(next);
		onMotionStatus?.(next);
	}, [controlledMotionStatus, onMotionStatus, setSharedMotionStatus]);
	const [pendingPowerAction, setPendingPowerAction] = useState<string | null>(null);
	const activeDriveFaultAxes = useMemo(
		() =>
			(driveFaults?.axes ?? []).filter(
				(axis) => axis.error_code !== 0 || axis.ds402_state === "Fault" || axis.ds402_state === "FaultReactionActive",
			),
		[driveFaults],
	);
	const commissioningDriveAxesByJoint = useMemo(() => {
		const mapping = new Map<number, DriveFaultAxis>();
		for (const axis of driveFaults?.axes ?? []) {
			if (typeof axis.logical_joint === "number" && Number.isFinite(axis.logical_joint)) {
				mapping.set(axis.logical_joint, axis);
			}
		}
		return mapping;
	}, [driveFaults]);
	const nativeHomeActiveAxisMask = driveFaults?.native_home_active_axis_mask ?? 0;
	const nativeHomeActiveAxes = useMemo(
		() =>
			(driveFaults?.axes ?? []).filter(
				(axis) => Boolean(axis.native_home_active) || ((nativeHomeActiveAxisMask & (1 << axis.axis)) !== 0),
			),
		[driveFaults, nativeHomeActiveAxisMask],
	);
	const nativeHomeBusy = nativeHomeActiveAxisMask !== 0 || nativeHomeActiveAxes.length > 0;
	const nativeHomeInProgressMessage = useMemo(() => {
		if (!nativeHomeBusy) {
			return null;
		}
		const labels = nativeHomeActiveAxes
			.map((axis) => (typeof axis.logical_joint === "number" ? `J${axis.logical_joint}` : `axis${axis.axis}`));
		const uniqueLabels = Array.from(new Set(labels));
		if (uniqueLabels.length === 0) {
			return "Drive-native home still running. Wait for the persistence step to finish before starting another home.";
		}
		const targets = uniqueLabels.join(", ");
		return `Drive-native home still running for ${targets}. Wait for the persistence step to finish before starting another home.`;
	}, [nativeHomeActiveAxes, nativeHomeBusy]);
	const driveControlBackend = (driveFaults?.servo_backend ?? activeServoBackend ?? "").trim().toLowerCase();
	const requiresExplicitDrivePower = driveControlBackend === "ethercat_rtcore";
	const drivePowerRequested = Boolean(
		driveFaults?.enable_requested
		?? ((driveFaults?.armed ?? 0) !== 0 || (driveFaults?.axis_enable_mask ?? 0) !== 0),
	);
	const totalDriveAxes = driveFaults?.num_axes ?? 0;
	const actualOpEnabledAxes = driveFaults?.op_enabled_axes ?? 0;
	const statuswordFeedbackAxes = driveFaults?.statusword_feedback_axes ?? 0;
	const isDrivePowerActive = actualOpEnabledAxes > 0 || (driveFaults?.driver_state ?? "").trim().toUpperCase() === "ACTIVE";
	const motionStateName = (motionStatus?.state ?? motionStatus?.execution?.state_name ?? "idle").trim().toLowerCase();
	const motionBusy = motionStateName === "accepted" || motionStateName === "queued" || motionStateName === "executing";
	const powerTransitionStatus = motionStatus as (MotionStatusResponse & PowerTransitionStatusView) | null;
	const powerTransitionExecution = motionStatus?.execution as (MotionExecutionPayload & PowerTransitionStatusView) | undefined;
	const powerTransitionBlockerDetails = (
		powerTransitionStatus?.power_transition_blocker_details
		?? powerTransitionExecution?.power_transition_blocker_details
		?? []
	) as PowerTransitionBlocker[];
	const powerTransitionSafeRaw = powerTransitionStatus?.safe_for_power_transition
		?? powerTransitionExecution?.safe_for_power_transition;
	const powerTransitionKnown = typeof powerTransitionSafeRaw === "boolean";
	const powerTransitionBlockerCodes = useMemo(
		() => powerTransitionBlockerDetails
			.map((detail) => (typeof detail.code === "string" ? detail.code.trim().toLowerCase() : ""))
			.filter((value) => value.length > 0),
		[powerTransitionBlockerDetails],
	);
	const showConservativeSafetyStatus = requiresExplicitDrivePower
		&& !isDrivePowerActive
		&& !drivePowerRequested
		&& pendingPowerAction === null;
	const onlySyncBlockers = powerTransitionBlockerCodes.length > 0
		&& powerTransitionBlockerCodes.every((code) => code === "not_synchronized");
	const [stableDisarmedPowerTransitionSafe, setStableDisarmedPowerTransitionSafe] = useState<boolean>(false);
	const stablePowerTransitionTimerRef = useRef<number | null>(null);
	useEffect(() => {
		if (stablePowerTransitionTimerRef.current !== null) {
			window.clearTimeout(stablePowerTransitionTimerRef.current);
			stablePowerTransitionTimerRef.current = null;
		}
		if (!powerTransitionKnown) {
			setStableDisarmedPowerTransitionSafe(false);
			return;
		}
		if (!showConservativeSafetyStatus) {
			setStableDisarmedPowerTransitionSafe(powerTransitionSafeRaw === true);
			return;
		}
		if (powerTransitionSafeRaw !== true) {
			setStableDisarmedPowerTransitionSafe(false);
			return;
		}
		if (stableDisarmedPowerTransitionSafe) {
			return;
		}
		stablePowerTransitionTimerRef.current = window.setTimeout(() => {
			setStableDisarmedPowerTransitionSafe(true);
			stablePowerTransitionTimerRef.current = null;
		}, POWER_TRANSITION_SAFE_STABLE_MS);
		return () => {
			if (stablePowerTransitionTimerRef.current !== null) {
				window.clearTimeout(stablePowerTransitionTimerRef.current);
				stablePowerTransitionTimerRef.current = null;
			}
		};
	}, [
		powerTransitionKnown,
		powerTransitionSafeRaw,
		showConservativeSafetyStatus,
		stableDisarmedPowerTransitionSafe,
	]);
	const powerTransitionSafe = powerTransitionKnown
		&& (
			showConservativeSafetyStatus
				? stableDisarmedPowerTransitionSafe
				: powerTransitionSafeRaw === true
		);
	const powerUpBlocked =
		activeDriveFaultAxes.length > 0
		|| (powerTransitionKnown ? powerTransitionSafe !== true : true);
	const powerUpBlockerMessage = activeDriveFaultAxes.length > 0
		? "Clear all drive faults before enabling the drives."
		: powerTransitionBlockerDetails.length > 0
			? powerTransitionBlockerDetails.map(formatPowerTransitionBlocker).join(" ")
			: "Waiting for the motion-state safety check before enabling the drives.";
	const reportRequestError = useCallback((err: unknown, fallback: string, silent = false) => {
		const msg = err instanceof Error ? err.message : fallback;
		if (!silent) {
			try {
				onError?.(msg);
			} catch {
				// ignore
			}
		}
		return msg;
	}, [onError]);
	const post = useCallback(async (path: string, body?: unknown) => {
		try {
			const res = await fetch(`${apiHost}${path}`, {
				method: "POST",
				headers: { "Content-Type": "application/json" },
				body: body ? JSON.stringify(body) : undefined,
			});
			if (!res.ok) {
				const msg = await readErrorMessage(res);
				throw new Error(msg || `${res.status} ${res.statusText}`);
			}
			const contentType = res.headers.get("content-type") ?? "";
			if (contentType.includes("application/json")) {
				return await res.json();
			}
			return null;
		} catch (err) {
			reportRequestError(err, "request failed");
			return null;
		}
	}, [apiHost, reportRequestError]);
	const refreshJointAngles = useCallback(async () => {
		try {
			const res = await fetch(`${apiHost}/info/joints`, {
				method: "GET",
				headers: { Accept: "application/json" },
			});
			if (!res.ok) {
				const msg = await readErrorMessage(res);
				throw new Error(msg || `${res.status} ${res.statusText}`);
			}
			const payload = await res.json() as JointInfoResponse;
			const nextAngles = preferredJointAnglesDeg(payload);
			if (!hasAnyFiniteJointAngles(nextAngles)) {
				return false;
			}
			if (hasAllFiniteJointAngles(nextAngles)) {
				onJointFeedback?.(
					nextAngles,
					typeof payload.gripper_deg === "number" ? payload.gripper_deg : undefined,
				);
			} else {
				onJointFeedback?.([], undefined);
			}
			return true;
		} catch (err) {
			reportRequestError(err, "Joint feedback unavailable.", true);
			return false;
		}
	}, [apiHost, onJointFeedback, reportRequestError]);
	const handleResetFaults = useCallback(async () => {
		if (pendingPowerAction || !requiresExplicitDrivePower) {
			return;
		}
		const confirmed = window.confirm(
			"Request a drive fault reset for all RTCore axes? Use this only when a driver is faulted.",
		);
		if (!confirmed) {
			return;
		}
		setPendingPowerAction("reset-faults");
		try {
			const result = await post("/control/reset-faults") as MotionStatusResponse | null;
			if (result) {
				setMotionStatus(result);
				await refreshJointAngles();
			}
		} finally {
			setPendingPowerAction(null);
		}
	}, [pendingPowerAction, post, refreshJointAngles, requiresExplicitDrivePower, setMotionStatus]);
	const handlePowerUpDrives = useCallback(async () => {
		if (pendingPowerAction || !requiresExplicitDrivePower) {
			return;
		}
		if (powerUpBlocked) {
			onError?.(powerUpBlockerMessage);
			return;
		}
		const confirmed = window.confirm(
			"Power up RTCore-controlled drives now? This will only proceed if the runtime is neutral, synchronized, and fault-free.",
		);
		if (!confirmed) {
			return;
		}
		setPendingPowerAction("power-up");
		try {
			const result = await post("/control/power-up") as MotionStatusResponse | null;
			if (result) {
				setMotionStatus(result);
				await refreshJointAngles();
			}
		} finally {
			setPendingPowerAction(null);
		}
	}, [
		onError,
		pendingPowerAction,
		post,
		powerUpBlocked,
		powerUpBlockerMessage,
		refreshJointAngles,
		requiresExplicitDrivePower,
		setMotionStatus,
	]);
	const handlePowerDownDrives = useCallback(async () => {
		if (pendingPowerAction || !requiresExplicitDrivePower) {
			return;
		}
		const confirmed = window.confirm(
			"Power down RTCore-controlled drives now? This will disable and disarm the configured axes.",
		);
		if (!confirmed) {
			return;
		}
		setPendingPowerAction("power-down");
		try {
			const result = await post("/control/power-down", { wait_for_idle: true }) as MotionStatusResponse | null;
			if (result) {
				setMotionStatus(result);
				await refreshJointAngles();
			}
		} finally {
			setPendingPowerAction(null);
		}
	}, [pendingPowerAction, post, refreshJointAngles, requiresExplicitDrivePower, setMotionStatus]);
	const driveStatusLabel = !requiresExplicitDrivePower
		? "AUTO"
		: pendingPowerAction === "power-up"
			? "ARMING"
			: pendingPowerAction === "power-down"
				? "DISARM"
				: isDrivePowerActive
					? "ARMED"
					: drivePowerRequested
						? "REQUESTED"
					: "DISARM";
	const motionStatusLabel = motionStateName === "faulted" || motionStateName === "aborted" || motionStateName === "underrun"
		? "FAULT"
		: motionBusy
			? "BUSY"
			: motionStateName === "completed"
				? "DONE"
				: formatMotionStateLabel(motionStateName);
	const safetyStatusPendingCheck = !powerTransitionKnown
		|| (
			showConservativeSafetyStatus
			&& (
				(powerTransitionSafeRaw === true && !stableDisarmedPowerTransitionSafe)
				|| (!powerTransitionSafe && onlySyncBlockers)
			)
		);
	const safetyStatusLabel = powerTransitionSafe
		? "SAFE"
		: safetyStatusPendingCheck
			? "CHECK"
			: "BLOCKED";
	const driveStatusTitle = requiresExplicitDrivePower
		? (
			driveFaults
				? `Drive power actual ${driveFaults.driver_state ?? "unknown"}. Backend ${driveFaults.servo_backend ?? "unknown"} | request ${drivePowerRequested ? "enable" : "none"} | EtherCAT ${driveFaults.ethercat_master_state ?? "unknown"} | RTCore ${driveFaults.rtcore_state ?? "unknown"} | op-enabled ${actualOpEnabledAxes}/${totalDriveAxes} | statusword feedback ${statuswordFeedbackAxes}/${totalDriveAxes}`
				: "Waiting for RTCore drive-state telemetry."
		)
		: "Drive power controls are only required for the EtherCAT RTCore backend.";
	const motionStatusTitle = motionStatus
		? `Motion ${formatMotionStateLabel(motionStateName)}. Source ${motionStatus.source_of_truth ?? "controller"} | scope ${motionStatus.completion_scope ?? "unknown"} | mode ${motionStatus.execution?.active_mode_name ?? "n/a"} | queue ${motionStatus.execution?.queue_depth ?? 0}/${motionStatus.execution?.queue_capacity ?? 0}`
		: "Waiting for controller motion status.";
	const safetyStatusTitle = powerTransitionSafe
		? "Runtime is neutral and synchronized; explicit drive enable is allowed."
		: safetyStatusPendingCheck
			? "Waiting for live feedback to remain synchronized long enough for a stable explicit drive-enable indication."
		: powerUpBlockerMessage;
	return (
		<div className={`flex min-w-0 h-full items-stretch gap-1.5 ${className ?? ""}`}>
			<span
				title={driveStatusTitle}
				className={`inline-flex shrink-0 h-full items-center gap-1 border px-2.5 text-[10px] font-semibold uppercase tracking-[0.16em] ${
					requiresExplicitDrivePower
						? (isDrivePowerActive
							? "border-emerald-500/40 bg-emerald-400/10 text-emerald-100"
							: "border-amber-500/40 bg-amber-400/10 text-amber-100")
						: "border-slate-600/70 bg-slate-900/70 text-slate-300"
				}`}
			>
				<Power size={11} />
				{driveStatusLabel}
			</span>
			<span
				title={motionStatusTitle}
				className={`inline-flex shrink-0 h-full items-center gap-1 border px-2.5 text-[10px] font-semibold uppercase tracking-[0.16em] ${
					motionStateName === "faulted" || motionStateName === "aborted" || motionStateName === "underrun"
						? "border-rose-500/40 bg-rose-400/10 text-rose-100"
						: motionBusy
							? "border-cyan-500/40 bg-cyan-400/10 text-cyan-100"
							: motionStateName === "completed"
								? "border-emerald-500/40 bg-emerald-400/10 text-emerald-100"
								: "border-slate-600/70 bg-slate-900/70 text-slate-300"
				}`}
			>
				<Activity size={11} />
				{motionStatusLabel}
			</span>
			<span
				title={safetyStatusTitle}
				className={`inline-flex shrink-0 h-full items-center gap-1 border px-2.5 text-[10px] font-semibold uppercase tracking-[0.16em] ${
					powerTransitionSafe
						? "border-emerald-500/40 bg-emerald-400/10 text-emerald-100"
						: safetyStatusPendingCheck
							? "border-slate-600/70 bg-slate-900/70 text-slate-300"
							: "border-amber-500/40 bg-amber-400/10 text-amber-100"
				}`}
			>
				{powerTransitionSafe ? <ShieldCheck size={11} /> : <ShieldAlert size={11} />}
				{safetyStatusLabel}
			</span>
			{activeDriveFaultAxes.length > 0 ? (
				<span
					title={`${activeDriveFaultAxes.length} active drive fault${activeDriveFaultAxes.length === 1 ? "" : "s"} detected. Use Reset to request a DS402 fault reset.`}
					className="inline-flex shrink-0 h-full items-center gap-1 border border-rose-500/40 bg-rose-400/10 px-2.5 text-[10px] font-semibold uppercase tracking-[0.16em] text-rose-100"
				>
					<AlertTriangle size={11} />
					{activeDriveFaultAxes.length}
				</span>
			) : null}
			{requiresExplicitDrivePower ? (
				<>
					<button
						type="button"
						onClick={() => {
							void handlePowerUpDrives();
						}}
						disabled={pendingPowerAction !== null || isDrivePowerActive || drivePowerRequested || powerUpBlocked}
						title={
							drivePowerRequested && !isDrivePowerActive
								? "Enable already requested; waiting for actual drive feedback."
								: powerUpBlocked
									? powerUpBlockerMessage
									: "Explicitly power up and enable RTCore-controlled axes"
						}
						className="shrink-0 h-full border border-cyan-500/40 bg-cyan-400/10 px-3 text-[10px] font-semibold uppercase tracking-[0.16em] text-cyan-100 transition hover:border-cyan-300/60 hover:bg-cyan-300/15 disabled:cursor-not-allowed disabled:opacity-50"
					>
						{pendingPowerAction === "power-up" ? "Powering..." : "Power Up"}
					</button>
					<button
						type="button"
						onClick={() => {
							void handlePowerDownDrives();
						}}
						disabled={pendingPowerAction !== null || (!isDrivePowerActive && !drivePowerRequested)}
						title="Explicitly power down and disarm RTCore-controlled axes"
						className="shrink-0 h-full border border-rose-500/40 bg-rose-400/10 px-3 text-[10px] font-semibold uppercase tracking-[0.16em] text-rose-100 transition hover:border-rose-300/60 hover:bg-rose-300/15 disabled:cursor-not-allowed disabled:opacity-50"
					>
						{pendingPowerAction === "power-down" ? "Powering Down..." : "Power Down"}
					</button>
					<button
						type="button"
						onClick={() => {
							void handleResetFaults();
						}}
						disabled={pendingPowerAction !== null}
						title="Request a DS402 fault reset for all RTCore-controlled axes"
						className="shrink-0 h-full border border-amber-500/40 bg-amber-400/10 px-3 text-[10px] font-semibold uppercase tracking-[0.16em] text-amber-100 transition hover:border-amber-300/60 hover:bg-amber-300/15 disabled:cursor-not-allowed disabled:opacity-50"
					>
						{pendingPowerAction === "reset-faults" ? "Resetting..." : "Reset"}
					</button>
				</>
			) : null}
		</div>
	);
}

export function ControlPanel({
	apiHost,
	driveFaults,
	activeServoBackend,
	onJointFeedback,
	onError,
	motionStatus: controlledMotionStatus,
	onMotionStatus,
	splitStatusSections = false,
	showStatusSections = true,
	showGripperPanel = true,
	showSoftwareZeroButton = false,
	controlsCollapsed = false,
	onToggleControlsCollapsed,
}: Props) {
	const liveState = useOptionalLiveState();
	const latestTelemetry = liveState?.latest ?? null;
	const isMonitorFresh = liveState?.isMonitorFresh ?? false;
	const telemetryStale = Boolean(latestTelemetry?.joint_feedback_stale);
	const telemetryStaleAge = typeof latestTelemetry?.joint_feedback_stale_age_s === "number"
		? latestTelemetry.joint_feedback_stale_age_s
		: null;
	const sharedMotionStatus = liveState?.motionStatus ?? null;
	const setSharedMotionStatus = liveState?.setMotionStatus;
	const [speedVal, setSpeedVal] = useState<number>(DEFAULT_SPEED_SLIDER); // 0..1000
	const speedMult = useMemo(() => expSliderToMultiplier(speedVal), [speedVal]);
	const [grip, setGrip] = useState<number>(0);
	const gripTimerRef = useRef<number | null>(null);
	const [jointAnglesDeg, setJointAnglesDeg] = useState<number[]>([]);
	const [jointFeedbackError, setJointFeedbackError] = useState<string | null>(null);
	const [jointStepDeg, setJointStepDeg] = useState<number>(1);
	const [customJointStepInput, setCustomJointStepInput] = useState<string>("");
	const [pendingJointAction, setPendingJointAction] = useState<string | null>(null);
	const lastJointStepCompletedAtRef = useRef(0);
	const [pendingMotionAction, setPendingMotionAction] = useState<string | null>(null);
	const [encoderRetentionExperimentId, setEncoderRetentionExperimentId] = useState<string | null>(null);
	const [commissioningStatus, setCommissioningStatus] = useState<CommissioningStatus | null>(null);
	const [commissioningExpanded, setCommissioningExpanded] = useState<boolean>(false);
	const [localMotionStatus, setLocalMotionStatus] = useState<MotionStatusResponse | null>(null);
	const motionStatus = controlledMotionStatus ?? sharedMotionStatus ?? localMotionStatus;
	const setMotionStatus = useCallback((next: MotionStatusResponse | null) => {
		if (controlledMotionStatus === undefined && !setSharedMotionStatus) {
			setLocalMotionStatus(next);
		}
		setSharedMotionStatus?.(next);
		onMotionStatus?.(next);
	}, [controlledMotionStatus, onMotionStatus, setSharedMotionStatus]);
	const activeDriveFaultAxes = useMemo(
		() =>
			(driveFaults?.axes ?? []).filter(
				(axis) => axis.error_code !== 0 || axis.ds402_state === "Fault" || axis.ds402_state === "FaultReactionActive",
			),
		[driveFaults],
	);
	const commissioningDriveAxesByJoint = useMemo(() => {
		const mapping = new Map<number, DriveFaultAxis>();
		for (const axis of driveFaults?.axes ?? []) {
			if (typeof axis.logical_joint === "number" && Number.isFinite(axis.logical_joint)) {
				mapping.set(axis.logical_joint, axis);
			}
		}
		return mapping;
	}, [driveFaults]);
	const nativeHomeActiveAxisMask = driveFaults?.native_home_active_axis_mask ?? 0;
	const nativeHomeActiveAxes = useMemo(
		() =>
			(driveFaults?.axes ?? []).filter(
				(axis) => Boolean(axis.native_home_active) || ((nativeHomeActiveAxisMask & (1 << axis.axis)) !== 0),
			),
		[driveFaults, nativeHomeActiveAxisMask],
	);
	const nativeHomeBusy = nativeHomeActiveAxisMask !== 0 || nativeHomeActiveAxes.length > 0;
	const nativeHomeInProgressMessage = useMemo(() => {
		if (!nativeHomeBusy) {
			return null;
		}
		const labels = nativeHomeActiveAxes
			.map((axis) => (typeof axis.logical_joint === "number" ? `J${axis.logical_joint}` : `axis${axis.axis}`));
		const uniqueLabels = Array.from(new Set(labels));
		if (uniqueLabels.length === 0) {
			return "Drive-native home still running. Wait for the persistence step to finish before starting another home.";
		}
		const targets = uniqueLabels.join(", ");
		return `Drive-native home still running for ${targets}. Wait for the persistence step to finish before starting another home.`;
	}, [nativeHomeActiveAxes, nativeHomeBusy]);
	const commissioningMessages = useMemo<CommissioningMessageEntry[]>(() => {
		const entries: CommissioningMessageEntry[] = [];
		if (jointFeedbackError) {
			entries.push({
				key: "joint-feedback",
				tone: "neutral",
				message: jointFeedbackError,
			});
		}
		if (commissioningStatus) {
			entries.push({
				key: "commissioning-status",
				tone: commissioningStatus.tone,
				message: commissioningStatus.message,
			});
		}
		if (nativeHomeInProgressMessage) {
			entries.push({
				key: "native-home-progress",
				tone: "warning",
				message: nativeHomeInProgressMessage,
			});
		}
		return entries;
	}, [commissioningStatus, jointFeedbackError, nativeHomeInProgressMessage]);
	const driveControlBackend = (driveFaults?.servo_backend ?? activeServoBackend ?? "").trim().toLowerCase();
	const requiresExplicitDrivePower = driveControlBackend === "ethercat_rtcore";
	const drivePowerRequested = Boolean(
		driveFaults?.enable_requested
		?? ((driveFaults?.armed ?? 0) !== 0 || (driveFaults?.axis_enable_mask ?? 0) !== 0),
	);
	const totalDriveAxes = driveFaults?.num_axes ?? 0;
	const actualOpEnabledAxes = driveFaults?.op_enabled_axes ?? 0;
	const statuswordFeedbackAxes = driveFaults?.statusword_feedback_axes ?? 0;
	const isDrivePowerActive = actualOpEnabledAxes > 0 || (driveFaults?.driver_state ?? "").trim().toUpperCase() === "ACTIVE";
	const drivePowerReady = !requiresExplicitDrivePower || isDrivePowerActive;
	const motionStateName = (motionStatus?.state ?? motionStatus?.execution?.state_name ?? "idle").trim().toLowerCase();
	const motionTrajectoryId = typeof motionStatus?.trajectory_id === "number" && motionStatus.trajectory_id > 0
		? motionStatus.trajectory_id
		: (typeof motionStatus?.execution?.active_traj_id === "number" && motionStatus.execution.active_traj_id > 0
			? motionStatus.execution.active_traj_id
			: null);
	const motionBusy = motionStateName === "accepted" || motionStateName === "queued" || motionStateName === "executing";
	const motionControlsBusy = motionBusy || pendingMotionAction !== null;
	const powerTransitionStatus = motionStatus as (MotionStatusResponse & PowerTransitionStatusView) | null;
	const powerTransitionExecution = motionStatus?.execution as (MotionExecutionPayload & PowerTransitionStatusView) | undefined;
	const powerTransitionBlockerDetails = (
		powerTransitionStatus?.power_transition_blocker_details
		?? powerTransitionExecution?.power_transition_blocker_details
		?? []
	) as PowerTransitionBlocker[];
	const powerTransitionSafe = powerTransitionStatus?.safe_for_power_transition ?? powerTransitionExecution?.safe_for_power_transition;
	const powerTransitionKnown = typeof powerTransitionSafe === "boolean";
	const powerUpBlocked =
		activeDriveFaultAxes.length > 0
		|| (powerTransitionKnown ? powerTransitionSafe !== true : true);
	const powerUpBlockerMessage = activeDriveFaultAxes.length > 0
		? "Clear all drive faults before enabling the drives."
		: powerTransitionBlockerDetails.length > 0
			? powerTransitionBlockerDetails.map(formatPowerTransitionBlocker).join(" ")
			: "Waiting for the motion-state safety check before enabling the drives.";
	// Realtime jog state
	const [jogEnabled, setJogEnabled] = useState<boolean>(false);
	const [deadman, setDeadman] = useState<boolean>(true);
	const [linBaseMmS, setLinBaseMmS] = useState<number>(50);
	const [angBaseDegS, setAngBaseDegS] = useState<number>(15);
	const [lastJogCommand, setLastJogCommand] = useState<JogCommandVector>([0, 0, 0, 0, 0, 0]);
	const driveControlsReferenceLabel = showStatusSections ? "Drive Power section above" : "header drive controls above";
	const jogTimerRef = useRef<number | null>(null);
	const jogKeepaliveTimerRef = useRef<number | null>(null);
	const sendJogTickRef = useRef<() => Promise<void>>(async () => {});
	const jogEnabledRef = useRef<boolean>(false);
	const jogHoldActiveRef = useRef<boolean>(false);
	const ignoreNextJogReleaseRef = useRef<boolean>(false);
	const deadmanRef = useRef<boolean>(true);
	const lastSentRef = useRef<JogCommandVector>([0, 0, 0, 0, 0, 0]);
	const lastSentAtRef = useRef<number>(0);
	const queuedJogStateRef = useRef<JogStatePayload | null>(null);
	const queuedJogStateSilentRef = useRef<boolean>(true);
	const jogStatePublishLoopRef = useRef<Promise<void> | null>(null);
	const jogOwnerIdRef = useRef<string>(createJogOwnerId());
	const jogSessionIdRef = useRef<string | null>(null);
	const jogSessionSeqRef = useRef<number>(-1);
	const jogSessionStatusRef = useRef<string>("idle");
	const pendingMotionActionRef = useRef<string | null>(null);
	const lastRequestErrorRef = useRef<string | null>(null);
	const keepaliveMs = 100;
	const jogTickIntervalMs = 50;
	const linCountsRef = useRef<{ x: number; y: number; z: number }>({ x: 0, y: 0, z: 0 });
	const angCountsRef = useRef<{ x: number; y: number; z: number }>({ x: 0, y: 0, z: 0 });
	const jointStepLabel = formatStepDegrees(jointStepDeg);

	const reportRequestError = useCallback((err: unknown, fallback: string, silent: boolean) => {
		const msg = (err as Error)?.message || fallback;
		lastRequestErrorRef.current = msg;
		if (!silent) {
			console.error("ControlPanel error:", err);
		}
		if (!silent) {
			try {
				onError?.(msg);
			} catch {
				// ignore
			}
		}
		return msg;
	}, [onError]);

	const post = useCallback(async (path: string, body?: unknown, silent = false) => {
		try {
			const res = await fetch(`${apiHost}${path}`, {
				method: "POST",
				headers: { "Content-Type": "application/json" },
				body: body ? JSON.stringify(body) : undefined,
			});
			if (!res.ok) {
				const msg = await readErrorMessage(res);
				throw new Error(msg || `${res.status} ${res.statusText}`);
			}
			lastRequestErrorRef.current = null;
			const contentType = res.headers.get("content-type") ?? "";
			if (contentType.includes("application/json")) {
				return await res.json();
			}
			return null;
		} catch (err) {
			reportRequestError(err, "request failed", silent);
			return null;
		}
	}, [apiHost, reportRequestError]);

	const getJson = useCallback(async <T,>(path: string, silent = false): Promise<T | null> => {
		try {
			const res = await fetch(`${apiHost}${path}`, {
				method: "GET",
				headers: { Accept: "application/json" },
			});
			if (!res.ok) {
				const msg = await readErrorMessage(res);
				throw new Error(msg || `${res.status} ${res.statusText}`);
			}
			lastRequestErrorRef.current = null;
			return (await res.json()) as T;
		} catch (err) {
			reportRequestError(err, "request failed", silent);
			return null;
		}
	}, [apiHost, reportRequestError]);

	const handleGripChange = useCallback((value: number) => {
		setGrip(value);
		if (gripTimerRef.current) {
			window.clearTimeout(gripTimerRef.current);
			gripTimerRef.current = null;
		}
		gripTimerRef.current = window.setTimeout(async () => {
			gripTimerRef.current = null;
			await post("/control/set-gripper", { angle_deg: value });
		}, 80);
	}, [post]);

	const refreshJointAngles = useCallback(async (silent = true) => {
		const payload = await getJson<JointInfoResponse>("/info/joints-detailed", silent);
		const nextAngles = preferredJointAnglesDeg(payload);
		if (!payload || !hasAnyFiniteJointAngles(nextAngles)) {
			setJointAnglesDeg([]);
			try {
				onJointFeedback?.([], undefined);
			} catch {
				// Ignore visualizer sync errors so commissioning feedback stays usable.
			}
			if (!silent) {
				setJointFeedbackError("Joint feedback unavailable.");
			}
			return false;
		}
		// `read_source` reports canonical/raw truth. The commissioning pane should still
		// show explicit operator display truth when the backend publishes it per-joint.
		setJointAnglesDeg(nextAngles);
		try {
			if (hasAllFiniteJointAngles(nextAngles)) {
				onJointFeedback?.(
					nextAngles,
					typeof payload.gripper_deg === "number" ? Number(payload.gripper_deg) : undefined,
				);
			} else {
				onJointFeedback?.([], undefined);
			}
		} catch {
			// Ignore visualizer sync errors so commissioning feedback stays usable.
		}
		setJointFeedbackError(null);
		return true;
	}, [getJson, onJointFeedback]);

	const waitForLiveJointFeedback = useCallback(async (
		initialDelayMs = 150,
		attempts = 6,
		intervalMs = 150,
	) => {
		const sleep = (delayMs: number) => new Promise<void>((resolve) => {
			window.setTimeout(resolve, delayMs);
		});
		const totalAttempts = Math.max(1, attempts);
		if (initialDelayMs > 0) {
			await sleep(initialDelayMs);
		}
		for (let attempt = 0; attempt < totalAttempts; attempt += 1) {
			const silent = attempt + 1 < totalAttempts;
			const ok = await refreshJointAngles(silent);
			if (ok) {
				return true;
			}
			if (attempt + 1 < totalAttempts && intervalMs > 0) {
				await sleep(intervalMs);
			}
		}
		return false;
	}, [refreshJointAngles]);

	useEffect(() => {
		const telemetryAnglesRad = preferredTelemetryJointAnglesRad(latestTelemetry);
		if (!isMonitorFresh || !hasAnyFiniteJointAngles(telemetryAnglesRad)) {
			return;
		}
		const nextAngles = telemetryAnglesRad.map((value) => (
			Number.isFinite(value) ? Number((value * 180) / Math.PI) : Number.NaN
		));
		setJointAnglesDeg((current) => {
			if (jointAngleArraysMatch(current, nextAngles)) {
				return current;
			}
			return nextAngles;
		});
		setJointFeedbackError(null);
	}, [isMonitorFresh, latestTelemetry]);

	// Boolean summary of "does the monitor stream already carry a usable
	// joint-angle snapshot?" — stable across SSE ticks that don't change the
	// availability flag.
	const hasFreshTelemetryJointAngles = useMemo(() => {
		if (!isMonitorFresh) {
			return false;
		}
		return hasAnyFiniteJointAngles(preferredTelemetryJointAnglesRad(latestTelemetry));
	}, [isMonitorFresh, latestTelemetry]);
	const shouldPollJointFeedback =
		!liveState || !liveState.isConnected || !isMonitorFresh;

	useEffect(() => {
		if (!shouldPollJointFeedback || hasFreshTelemetryJointAngles) {
			return;
		}
		let disposed = false;
		let inFlight = false;
		const tick = async () => {
			if (disposed || inFlight) {
				return;
			}
			if (typeof document !== "undefined" && document.visibilityState !== "visible") {
				return;
			}
			inFlight = true;
			const ok = await refreshJointAngles(true);
			inFlight = false;
			if (disposed) {
				return;
			}
			if (!ok) {
				setJointFeedbackError("Waiting for joint feedback...");
			}
		};
		void tick();
		const timer = window.setInterval(() => {
			void tick();
		}, STANDALONE_JOINT_FEEDBACK_POLL_MS);
		return () => {
			disposed = true;
			window.clearInterval(timer);
		};
	}, [hasFreshTelemetryJointAngles, refreshJointAngles, shouldPollJointFeedback]);

	useEffect(() => {
		if (controlledMotionStatus !== undefined || liveState) {
			return;
		}
		let disposed = false;
		let inFlight = false;
		const tick = async () => {
			if (disposed || inFlight) {
				return;
			}
			inFlight = true;
			const payload = await getJson<MotionStatusResponse>("/control/motion-status", true);
			inFlight = false;
			if (disposed || !payload) {
				return;
			}
			setMotionStatus(payload);
		};
		void tick();
		const timer = window.setInterval(() => {
			void tick();
		}, 200);
		return () => {
			disposed = true;
			window.clearInterval(timer);
		};
	}, [controlledMotionStatus, getJson, liveState, setMotionStatus]);

	// ------------------------
	// Realtime jog helpers
	// ------------------------
	const computeJogVector = useCallback(() => {
		const lc = linCountsRef.current;
		const ac = angCountsRef.current;
		const baseMS = (linBaseMmS / 1000.0) * speedMult;
		const baseDegS = angBaseDegS * speedMult;
		const vx = (lc.x > 0 ? 1 : lc.x < 0 ? -1 : 0) * baseMS;
		const vy = (lc.y > 0 ? 1 : lc.y < 0 ? -1 : 0) * baseMS;
		const vz = (lc.z > 0 ? 1 : lc.z < 0 ? -1 : 0) * baseMS;
		const vroll = (ac.x > 0 ? 1 : ac.x < 0 ? -1 : 0) * baseDegS;
		const vpitch = (ac.y > 0 ? 1 : ac.y < 0 ? -1 : 0) * baseDegS;
		const vyaw = (ac.z > 0 ? 1 : ac.z < 0 ? -1 : 0) * baseDegS;
		if (!deadman) return [0, 0, 0, 0, 0, 0] as [number, number, number, number, number, number];
		return [vx, vy, vz, vroll, vpitch, vyaw] as [number, number, number, number, number, number];
	}, [linBaseMmS, angBaseDegS, speedMult, deadman]);

	const effectiveLinearMS = useMemo(() => (linBaseMmS / 1000.0) * speedMult, [linBaseMmS, speedMult]);
	const effectiveAngularDegS = useMemo(() => angBaseDegS * speedMult, [angBaseDegS, speedMult]);

	const hasActiveJogInput = useCallback(() => {
		const linear = linCountsRef.current;
		const angular = angCountsRef.current;
		return linear.x !== 0 || linear.y !== 0 || linear.z !== 0 || angular.x !== 0 || angular.y !== 0 || angular.z !== 0;
	}, []);

	const buildJogStatePayload = useCallback((
		commandPayload: JogCommandVector,
		overrides?: Partial<Pick<JogStatePayload, "active" | "deadman" | "stop_reason">>,
	): JogStatePayload => ({
		active: overrides?.active ?? jogHoldActiveRef.current,
		deadman: overrides?.deadman ?? deadmanRef.current,
		vx: commandPayload[0],
		vy: commandPayload[1],
		vz: commandPayload[2],
		v_roll: commandPayload[3],
		v_pitch: commandPayload[4],
		v_yaw: commandPayload[5],
		stop_reason: overrides?.stop_reason,
	}), []);

	const resetJogSessionTracking = useCallback(() => {
		jogSessionIdRef.current = null;
		jogSessionSeqRef.current = -1;
		jogSessionStatusRef.current = "idle";
	}, []);

	const clearJogKeepaliveTimer = useCallback(() => {
		if (jogKeepaliveTimerRef.current) {
			window.clearTimeout(jogKeepaliveTimerRef.current);
			jogKeepaliveTimerRef.current = null;
		}
	}, []);

	const scheduleJogKeepalive = useCallback(() => {
		clearJogKeepaliveTimer();
		if (!jogHoldActiveRef.current || !hasActiveJogInput()) {
			return;
		}
		jogKeepaliveTimerRef.current = window.setTimeout(() => {
			jogKeepaliveTimerRef.current = null;
			sendJogTickRef.current().catch(() => {});
		}, keepaliveMs);
	}, [clearJogKeepaliveTimer, hasActiveJogInput, keepaliveMs]);

	const enqueueJogState = useCallback((payload: JogStatePayload, silent = true) => {
		const hadPendingPayload = queuedJogStateRef.current !== null;
		queuedJogStateRef.current = payload;
		queuedJogStateSilentRef.current = hadPendingPayload ? (queuedJogStateSilentRef.current && silent) : silent;
		if (!jogStatePublishLoopRef.current) {
			jogStatePublishLoopRef.current = (async () => {
				try {
					while (queuedJogStateRef.current) {
						const nextPayload = queuedJogStateRef.current;
						const nextSilent = queuedJogStateSilentRef.current;
						queuedJogStateRef.current = null;
						queuedJogStateSilentRef.current = true;
						const nextCommand: JogCommandVector = [
							nextPayload.vx,
							nextPayload.vy,
							nextPayload.vz,
							nextPayload.v_roll,
							nextPayload.v_pitch,
							nextPayload.v_yaw,
						];
						const commandChanged =
							nextCommand[0] !== lastSentRef.current[0] ||
							nextCommand[1] !== lastSentRef.current[1] ||
							nextCommand[2] !== lastSentRef.current[2] ||
							nextCommand[3] !== lastSentRef.current[3] ||
							nextCommand[4] !== lastSentRef.current[4] ||
							nextCommand[5] !== lastSentRef.current[5];
						lastSentRef.current = nextCommand;
						lastSentAtRef.current = Date.now();
						if (commandChanged || !nextPayload.active) {
							setLastJogCommand(nextCommand);
						}
						if (!nextPayload.active) {
							if (jogHoldActiveRef.current || hasActiveJogInput()) {
								continue;
							}
							const activeSessionId = jogSessionIdRef.current;
							if (activeSessionId) {
								await post("/control/jog/session/stop", {
									session_id: activeSessionId,
									owner_id: jogOwnerIdRef.current,
									reason: nextPayload.stop_reason ?? "ui-release",
								}, nextSilent);
							}
							resetJogSessionTracking();
							continue;
						}
						if (!jogSessionIdRef.current && isZeroJogPayload(nextPayload)) {
							jogSessionStatusRef.current = "idle";
							continue;
						}
						const nextSeq = jogSessionSeqRef.current + 1;
						jogSessionSeqRef.current = nextSeq;
						const requestBody = {
							owner_id: jogOwnerIdRef.current,
							seq: nextSeq,
							deadman: nextPayload.deadman,
							lease_timeout_s: JOG_SESSION_LEASE_TIMEOUT_S,
							vx: nextPayload.vx,
							vy: nextPayload.vy,
							vz: nextPayload.vz,
							v_roll: nextPayload.v_roll,
							v_pitch: nextPayload.v_pitch,
							v_yaw: nextPayload.v_yaw,
						};
						const response = (jogSessionIdRef.current
							? await post(
								"/control/jog/session/update",
								{ session_id: jogSessionIdRef.current, ...requestBody },
								nextSilent,
							)
							: await post("/control/jog/session/start", requestBody, nextSilent)) as JogSessionResponse | null;
						const session = response?.session;
						if (session?.session_id) {
							jogSessionIdRef.current = session.session_id;
							jogSessionSeqRef.current = typeof session.last_seq_received === "number"
								? session.last_seq_received
								: nextSeq;
							jogSessionStatusRef.current = session.state ?? "active";
							if (nextPayload.active) {
								scheduleJogKeepalive();
							} else {
								clearJogKeepaliveTimer();
							}
						} else if (isJogSessionTerminalError(lastRequestErrorRef.current)) {
							const errorCode = getJogSessionErrorCode(lastRequestErrorRef.current);
							if (isJogSessionRecoverableError(errorCode)) {
								resetJogSessionTracking();
								if (nextPayload.active && hasActiveJogInput()) {
									const currentVector = computeJogVector();
									queuedJogStateRef.current = buildJogStatePayload(currentVector, {
										active: true,
										deadman: deadmanRef.current,
									});
									queuedJogStateSilentRef.current = nextSilent;
								}
								continue;
							}
							const zeroCommand: JogCommandVector = [0, 0, 0, 0, 0, 0];
							setJogEnabled(false);
							if (jogTimerRef.current) {
								window.clearInterval(jogTimerRef.current);
								jogTimerRef.current = null;
							}
							linCountsRef.current = { x: 0, y: 0, z: 0 };
							angCountsRef.current = { x: 0, y: 0, z: 0 };
							queuedJogStateRef.current = null;
							queuedJogStateSilentRef.current = true;
							setLastJogCommand(zeroCommand);
							lastSentRef.current = zeroCommand;
							lastSentAtRef.current = Date.now();
							jogEnabledRef.current = false;
							jogHoldActiveRef.current = false;
							clearJogKeepaliveTimer();
							resetJogSessionTracking();
						}
					}
				} finally {
					jogStatePublishLoopRef.current = null;
				}
			})();
		}
		return jogStatePublishLoopRef.current ?? Promise.resolve();
	}, [buildJogStatePayload, clearJogKeepaliveTimer, computeJogVector, hasActiveJogInput, post, resetJogSessionTracking, scheduleJogKeepalive]);

	const sendJogTick = useCallback(async () => {
		if (!jogHoldActiveRef.current) {
			return;
		}
		const now = Date.now();
		const v = computeJogVector();
		const last = lastSentRef.current;
		const hasSession = Boolean(jogSessionIdRef.current);
		const isZeroVector =
			v[0] === 0 && v[1] === 0 && v[2] === 0 &&
			v[3] === 0 && v[4] === 0 && v[5] === 0;
		const changed =
			v[0] !== last[0] || v[1] !== last[1] || v[2] !== last[2] ||
			v[3] !== last[3] || v[4] !== last[4] || v[5] !== last[5];
		if (!changed && now - lastSentAtRef.current < keepaliveMs) {
			return;
		}
		if (!changed && isZeroVector && !hasSession) {
			return;
		}
		const commandPayload: JogCommandVector = [
			Number(v[0].toFixed(6)),
			Number(v[1].toFixed(6)),
			Number(v[2].toFixed(6)),
			Number(v[3].toFixed(3)),
			Number(v[4].toFixed(3)),
			Number(v[5].toFixed(3)),
		];
		void enqueueJogState(buildJogStatePayload(commandPayload), true);
	}, [buildJogStatePayload, computeJogVector, enqueueJogState]);

	// Keep the interval callback hot-swapped with latest slider/deadman/base values.
	useEffect(() => {
		sendJogTickRef.current = sendJogTick;
	}, [sendJogTick]);

	useEffect(() => {
		jogEnabledRef.current = jogEnabled;
	}, [jogEnabled]);

	useEffect(() => {
		deadmanRef.current = deadman;
	}, [deadman]);

	const stopJogPolling = useCallback(() => {
		if (jogTimerRef.current) {
			window.clearInterval(jogTimerRef.current);
			jogTimerRef.current = null;
		}
		clearJogKeepaliveTimer();
	}, [clearJogKeepaliveTimer]);

	const ensureJogPolling = useCallback(() => {
		if (jogTimerRef.current) {
			return;
		}
		jogTimerRef.current = window.setInterval(() => {
			sendJogTickRef.current().catch(() => {});
		}, jogTickIntervalMs);
	}, [jogTickIntervalMs]);

	const stopJogLocally = useCallback((options?: { clearSessionTracking?: boolean; disarm?: boolean; clearQueuedState?: boolean }) => {
		const clearSessionTracking = options?.clearSessionTracking ?? false;
		const disarm = options?.disarm ?? false;
		const clearQueuedState = options?.clearQueuedState ?? false;
		const zeroCommand: JogCommandVector = [0, 0, 0, 0, 0, 0];
		stopJogPolling();
		linCountsRef.current = { x: 0, y: 0, z: 0 };
		angCountsRef.current = { x: 0, y: 0, z: 0 };
		if (clearQueuedState) {
			queuedJogStateRef.current = null;
			queuedJogStateSilentRef.current = true;
		}
		setLastJogCommand(zeroCommand);
		lastSentRef.current = zeroCommand;
		lastSentAtRef.current = Date.now();
		jogHoldActiveRef.current = false;
		if (disarm) {
			setJogEnabled(false);
			jogEnabledRef.current = false;
		}
		if (clearSessionTracking) {
			resetJogSessionTracking();
		}
	}, [resetJogSessionTracking, stopJogPolling]);

	const postJsonBestEffort = useCallback((path: string, body?: unknown) => {
		const payload = body === undefined ? "" : JSON.stringify(body);
		if (typeof navigator !== "undefined" && typeof navigator.sendBeacon === "function") {
			try {
				const blob = new Blob([payload], { type: "application/json" });
				if (navigator.sendBeacon(`${apiHost}${path}`, blob)) {
					return;
				}
			} catch {
				// Fall through to fetch keepalive.
			}
		}
		if (typeof fetch === "function") {
			void fetch(`${apiHost}${path}`, {
				method: "POST",
				headers: { "Content-Type": "application/json" },
				body: payload || undefined,
				keepalive: true,
			}).catch(() => {});
		}
	}, [apiHost]);

	const requestJogFailsafeStop = useCallback(() => {
		const sessionId = jogSessionIdRef.current;
		if (!jogEnabledRef.current && !jogHoldActiveRef.current && !sessionId) {
			return;
		}
		stopJogLocally({ clearQueuedState: true, disarm: true });
		if (sessionId) {
			postJsonBestEffort("/control/jog/session/stop", {
				session_id: sessionId,
				owner_id: jogOwnerIdRef.current,
				reason: "pagehide",
			});
		}
		resetJogSessionTracking();
	}, [postJsonBestEffort, resetJogSessionTracking, stopJogLocally]);

	useEffect(() => {
		const handlePageHide = () => {
			requestJogFailsafeStop();
		};
		const handleVisibilityChange = () => {
			if (typeof document !== "undefined" && document.visibilityState === "hidden") {
				requestJogFailsafeStop();
			}
		};
		window.addEventListener("pagehide", handlePageHide);
		window.addEventListener("beforeunload", handlePageHide);
		document.addEventListener("visibilitychange", handleVisibilityChange);
		return () => {
			document.removeEventListener("visibilitychange", handleVisibilityChange);
			window.removeEventListener("beforeunload", handlePageHide);
			window.removeEventListener("pagehide", handlePageHide);
			stopJogPolling();
			if (jogEnabledRef.current || jogHoldActiveRef.current || jogSessionIdRef.current) {
				const sessionId = jogSessionIdRef.current;
				if (sessionId) {
					postJsonBestEffort("/control/jog/session/stop", {
						session_id: sessionId,
						owner_id: jogOwnerIdRef.current,
						reason: "component-unmount",
					});
				}
				resetJogSessionTracking();
			}
		};
	}, [postJsonBestEffort, requestJogFailsafeStop, resetJogSessionTracking, stopJogPolling]);

	const armJogMode = useCallback(() => {
		if (jogEnabledRef.current) {
			return;
		}
		setJogEnabled(true);
		jogEnabledRef.current = true;
	}, []);

	const releaseJogHold = useCallback(async (reason = "ui-release") => {
		stopJogLocally();
		await enqueueJogState(buildJogStatePayload([0, 0, 0, 0, 0, 0], {
			active: false,
			deadman: false,
			stop_reason: reason,
		}), true);
	}, [buildJogStatePayload, enqueueJogState, stopJogLocally]);

	const stopJog = useCallback(async (reason = "ui-disarm") => {
		const shouldSendStop = jogHoldActiveRef.current || Boolean(jogSessionIdRef.current) || Boolean(queuedJogStateRef.current);
		stopJogLocally({ disarm: true });
		try {
			if (jogStatePublishLoopRef.current) {
				await jogStatePublishLoopRef.current;
			}
		} catch {
			// Ignore prior publish failure and still try the stop request.
		}
		if (shouldSendStop) {
			await enqueueJogState(buildJogStatePayload([0, 0, 0, 0, 0, 0], {
				active: false,
				deadman: false,
				stop_reason: reason,
			}), false);
		} else {
			resetJogSessionTracking();
		}
	}, [buildJogStatePayload, enqueueJogState, resetJogSessionTracking, stopJogLocally]);

	const runDiscreteMotionCommand = useCallback(async <T,>(actionKey: string, task: () => Promise<T>) => {
		if (motionBusy || pendingMotionActionRef.current) {
			return null as T | null;
		}
		pendingMotionActionRef.current = actionKey;
		setPendingMotionAction(actionKey);
		try {
			return await task();
		} finally {
			pendingMotionActionRef.current = null;
			setPendingMotionAction((current) => (current === actionKey ? null : current));
		}
	}, [motionBusy]);

	const onDeadmanToggle = useCallback(async (enabled: boolean) => {
		setDeadman(enabled);
		deadmanRef.current = enabled;
		if (jogHoldActiveRef.current) {
			const vector = enabled ? computeJogVector() : ([0, 0, 0, 0, 0, 0] as JogCommandVector);
			const commandPayload: JogCommandVector = [
				Number(vector[0].toFixed(6)),
				Number(vector[1].toFixed(6)),
				Number(vector[2].toFixed(6)),
				Number(vector[3].toFixed(3)),
				Number(vector[4].toFixed(3)),
				Number(vector[5].toFixed(3)),
			];
			await enqueueJogState(buildJogStatePayload(commandPayload, { active: true, deadman: enabled }), false);
		}
	}, [buildJogStatePayload, computeJogVector, enqueueJogState]);
	const onDebugToggle = useCallback(async (enabled: boolean) => {
		await post("/control/jog/debug", { enabled });
	}, [post]);
	const applyCustomJointStep = useCallback(() => {
		const trimmed = customJointStepInput.trim();
		if (!trimmed) {
			return false;
		}
		const parsed = Number(trimmed);
		if (!Number.isFinite(parsed) || Math.abs(parsed) < MIN_CUSTOM_JOINT_STEP_DEG) {
			setCommissioningStatus({
				tone: "error",
				message: `Enter a custom joint step of at least ${MIN_CUSTOM_JOINT_STEP_DEG} deg.`,
			});
			return false;
		}
		const normalized = Number(Math.abs(parsed).toFixed(3));
		setJointStepDeg(normalized);
		setCustomJointStepInput(formatStepDegrees(normalized));
		setCommissioningStatus({
			tone: "info",
			message: `Custom joint step set to +/-${formatStepDegrees(normalized)} deg.`,
		});
		return true;
	}, [customJointStepInput]);

	const handleJointStep = useCallback(async (jointIndex: number, direction: 1 | -1) => {
		if (pendingJointAction) {
			return;
		}
		if (performance.now() - lastJointStepCompletedAtRef.current < JOINT_STEP_DEBOUNCE_MS) {
			return;
		}
		const jointNumber = jointIndex + 1;
		const signedStepDeg = Number((jointStepDeg * direction).toFixed(3));
		const signedStepLabel = formatStepDegrees(Math.abs(signedStepDeg));
		setPendingJointAction(`jog-${jointIndex}`);
		setCommissioningStatus({
			tone: "info",
			message: `Jogging J${jointNumber} by ${signedStepDeg > 0 ? "+" : "-"}${signedStepLabel} deg...`,
		});
		try {
			if (jogEnabled) {
				await stopJog();
			}
			const result = await post("/control/joint-jog", {
				joint: jointNumber,
				delta_deg: signedStepDeg,
				wait_for_idle: true,
			}) as MotionStatusResponse | null;
			if (result) {
				setMotionStatus(result);
				const refreshed = await refreshJointAngles(false);
				const driveFaulted = driveFaults?.physical_state === "FAULTED";
				const opEnabledAxes = driveFaults?.op_enabled_axes ?? 0;
				const executionState = (result.state ?? result.execution?.state_name ?? "accepted").trim().toLowerCase();
				const queuedTrajectoryId = typeof result.trajectory_id === "number" && result.trajectory_id > 0
					? result.trajectory_id
					: null;
				if (result.command_acknowledged) {
					if (result.wait_for_idle_requested && result.waited_for_idle === false) {
						setCommissioningStatus({
							tone: "error",
							message: `J${jointNumber} jog was accepted but did not report idle before timeout. Check motion status before sending another jog.`,
						});
						lastJointStepCompletedAtRef.current = performance.now();
						return;
					}
					if (driveFaulted) {
						setCommissioningStatus({
							tone: "error",
							message: `J${jointNumber} setpoint was accepted, but the drives are faulted. Clear faults before expecting motion.`,
						});
					} else if (opEnabledAxes <= 0) {
						setCommissioningStatus({
							tone: "info",
							message: `J${jointNumber} setpoint was accepted, but no axes are operation-enabled. Verify drive state before expecting motion.`,
						});
					} else if (refreshed) {
						setCommissioningStatus({
							tone: "success",
							message: queuedTrajectoryId !== null
								? `J${jointNumber} accepted as RTCore trajectory ${queuedTrajectoryId} (${executionState}). Live joint feedback refreshed.`
								: `J${jointNumber} command accepted with state ${executionState}. Live joint feedback refreshed.`,
						});
					} else {
						setCommissioningStatus({
							tone: "info",
							message: queuedTrajectoryId !== null
								? `J${jointNumber} accepted as RTCore trajectory ${queuedTrajectoryId} (${executionState}), but live feedback refresh did not succeed yet.`
								: `J${jointNumber} command accepted with state ${executionState}, but live feedback refresh did not succeed yet.`,
						});
					}
				} else {
					setCommissioningStatus({
						tone: "error",
						message: `Failed to confirm backend acceptance for J${jointNumber}. Check controller connectivity and drive state.`,
					});
				}
				lastJointStepCompletedAtRef.current = performance.now();
			} else {
				const requestError = lastRequestErrorRef.current?.trim();
				setCommissioningStatus({
					tone: "error",
					message: requestError
						? `Failed to jog J${jointNumber}: ${requestError}`
						: `Failed to jog J${jointNumber}. Check API/controller connectivity.`,
				});
			}
		} finally {
			setPendingJointAction(null);
		}
	}, [pendingJointAction, jogEnabled, stopJog, post, jointStepDeg, refreshJointAngles, driveFaults]);

	const handleJointZero = useCallback(async (jointIndex: number) => {
		if (pendingJointAction) {
			return;
		}
		const jointNumber = jointIndex + 1;
		const angleLabel = Number.isFinite(jointAnglesDeg[jointIndex] ?? Number.NaN)
			? `${jointAnglesDeg[jointIndex].toFixed(2)} deg`
			: "current feedback";
		const confirmed = window.confirm(
			`Capture J${jointNumber} at ${angleLabel} as logical zero?`,
		);
		if (!confirmed) {
			return;
		}
		setPendingJointAction(`zero-${jointIndex}`);
		setCommissioningStatus({
			tone: "info",
			message: `Capturing J${jointNumber} at ${angleLabel} as logical zero...`,
		});
		try {
			if (jogEnabled) {
				await stopJog();
			}
			const result = await post("/control/zero-joint", { joint: jointNumber });
			if (result) {
				const refreshed = await refreshJointAngles(false);
				setCommissioningStatus(
					refreshed
						? {
							tone: "success",
							message: `Captured J${jointNumber} logical zero. Live feedback now reflects the updated offset.`,
						}
						: {
							tone: "error",
							message: `Captured J${jointNumber} logical zero, but live feedback refresh did not succeed.`,
						},
				);
			} else {
				setCommissioningStatus({
					tone: "error",
					message: `Failed to capture logical zero for J${jointNumber}.`,
				});
			}
		} finally {
			setPendingJointAction(null);
		}
	}, [pendingJointAction, jointAnglesDeg, jogEnabled, stopJog, post, refreshJointAngles]);

	const handleJointNativeHome = useCallback(async (jointIndex: number) => {
		if (pendingJointAction) {
			return;
		}
		const jointNumber = jointIndex + 1;
		const angleLabel = Number.isFinite(jointAnglesDeg[jointIndex] ?? Number.NaN)
			? `${jointAnglesDeg[jointIndex].toFixed(2)} deg`
			: "current live drive feedback";
		const confirmed = window.confirm(
			`Capture J${jointNumber} at ${angleLabel} as the drive-native home position? This runs the drive's commissioning homing transaction, leaves only that homed axis disabled afterward, and may require an explicit Power Up before you move it again.`,
		);
		if (!confirmed) {
			return;
		}
		setPendingJointAction(`native-home-${jointIndex}`);
		setCommissioningStatus({
			tone: "info",
			message: `Running drive-native commissioning home for J${jointNumber} at ${angleLabel}...`,
		});
		try {
			if (jogEnabled) {
				await stopJog();
			}
			const result = await post("/control/home-joint-native", { joint: jointNumber }) as NativeHomeResponse | null;
			if (result) {
				const refreshed = await waitForLiveJointFeedback();
				const accepted = result.accepted !== false;
				const verified = result.verified === true;
				const resultMessage = typeof result.message === "string" && result.message.trim().length > 0
					? result.message.trim()
					: null;
				if (accepted && verified) {
					setCommissioningStatus(
						refreshed
							? {
								tone: "success",
								message: resultMessage
									?? `Drive-native commissioning home verified for J${jointNumber}. Live feedback refreshed from the new encoder reference. The homed axis remains disabled until you explicitly power it back up.`,
							}
							: {
								tone: "warning",
								message: resultMessage
									?? `Drive-native commissioning home verified for J${jointNumber}, but the post-home live feedback did not refresh yet. Keep that axis disabled and confirm the pose before powering it back up.`,
							},
					);
				} else if (accepted) {
					setCommissioningStatus({
						tone: "warning",
						message: resultMessage
							?? (
								refreshed
									? `Drive-native commissioning home for J${jointNumber} was accepted, but verification is still pending. Live feedback refreshed; keep the axis disabled until the drive-home status settles.`
									: `Drive-native commissioning home for J${jointNumber} was accepted, but verification is still pending and live feedback did not refresh yet. Keep that axis disabled until the drive-home status settles.`
							),
					});
				} else {
					setCommissioningStatus({
						tone: "error",
						message: resultMessage
							?? `Drive-native commissioning home failed for J${jointNumber}. Keep that axis disabled and inspect the drive-home status before retrying.`,
					});
				}
			} else {
				const requestError = lastRequestErrorRef.current?.trim();
				setCommissioningStatus({
					tone: "error",
					message: requestError
						? `Failed to request drive-native home for J${jointNumber}: ${requestError}`
						: `Failed to request drive-native home for J${jointNumber}.`,
				});
			}
		} finally {
			setPendingJointAction(null);
		}
	}, [pendingJointAction, jointAnglesDeg, jogEnabled, stopJog, post, waitForLiveJointFeedback]);

	const handleRefreshJointFeedback = useCallback(async () => {
		setCommissioningStatus({
			tone: "info",
			message: "Refreshing live joint feedback...",
		});
		const refreshed = await refreshJointAngles(false);
		setCommissioningStatus(
			refreshed
				? {
					tone: "success",
					message: drivePowerReady
						? "Live joint feedback refreshed. You can now jog and capture zero from the current pose."
						: "Live joint feedback refreshed. Drives remain disarmed until you explicitly power them up.",
				}
				: {
					tone: "error",
					message: "Joint feedback unavailable. Do not capture zero until live angles are visible.",
				},
		);
	}, [drivePowerReady, refreshJointAngles]);

	const handleResetFaults = useCallback(async (jointNumber?: number | null) => {
		if (pendingJointAction) {
			return;
		}
		const isSingleJoint = typeof jointNumber === "number" && Number.isFinite(jointNumber);
		const targetLabel = isSingleJoint ? `J${jointNumber}` : "all RTCore axes";
		const confirmed = window.confirm(
			`Request a drive fault reset for ${targetLabel}? Use this only when a driver is faulted.`,
		);
		if (!confirmed) {
			return;
		}
		setPendingJointAction(isSingleJoint ? `reset-faults-${jointNumber}` : "reset-faults-all");
		setCommissioningStatus({
			tone: "info",
			message: isSingleJoint
				? `Requesting RTCore drive fault reset for J${jointNumber}...`
				: "Requesting RTCore drive fault reset for all axes...",
		});
		try {
			const result = await post(
				"/control/reset-faults",
				isSingleJoint ? { joint: jointNumber } : undefined,
			) as MotionStatusResponse | null;
			if (result) {
				setMotionStatus(result);
				const refreshed = await refreshJointAngles(false);
				setCommissioningStatus(
					refreshed
						? {
							tone: "success",
							message: isSingleJoint
								? `Drive fault reset requested for J${jointNumber}. Drives remain disarmed until an explicit safe power-up. Live joint feedback refreshed.`
								: "Drive fault reset requested for all axes. Drives remain disarmed until an explicit safe power-up. Live joint feedback refreshed.",
						}
						: {
							tone: "success",
							message: isSingleJoint
								? `Drive fault reset requested for J${jointNumber}. Drives remain disarmed until an explicit safe power-up. Refresh joint feedback after the drive recovers.`
								: "Drive fault reset requested for all axes. Drives remain disarmed until an explicit safe power-up. Refresh joint feedback after the drives recover.",
						},
				);
			} else {
				setCommissioningStatus({
					tone: "error",
					message: isSingleJoint
						? `Failed to request a drive fault reset for J${jointNumber}.`
						: "Failed to request a drive fault reset.",
				});
			}
		} finally {
			setPendingJointAction(null);
		}
	}, [pendingJointAction, post, refreshJointAngles, setMotionStatus]);

	const handlePowerUpDrives = useCallback(async () => {
		if (pendingJointAction) {
			return;
		}
		if (powerUpBlocked) {
			setCommissioningStatus({
				tone: "error",
				message: powerUpBlockerMessage,
			});
			return;
		}
		const confirmed = window.confirm(
			"Power up RTCore-controlled drives now? This will only proceed if the runtime is neutral, synchronized, and fault-free.",
		);
		if (!confirmed) {
			return;
		}
		setPendingJointAction("power-up");
		setCommissioningStatus({
			tone: "info",
			message: "Requesting drive power-up...",
		});
		try {
			const result = await post("/control/power-up") as MotionStatusResponse | null;
			if (result) {
				setMotionStatus(result);
				const refreshed = await refreshJointAngles(false);
				setCommissioningStatus(
					refreshed
						? {
							tone: "success",
							message: "Drive power-up requested after a safe-state check. Live joint feedback refreshed.",
						}
						: {
							tone: "success",
							message: "Drive power-up requested after a safe-state check. Refresh joint feedback after the axes finish enabling.",
						},
				);
			} else {
				setCommissioningStatus({
					tone: "error",
					message: "Failed to request drive power-up.",
				});
			}
		} finally {
			setPendingJointAction(null);
		}
	}, [pendingJointAction, post, powerUpBlocked, powerUpBlockerMessage, refreshJointAngles, setMotionStatus]);

	const handlePowerDownDrives = useCallback(async () => {
		if (pendingJointAction) {
			return;
		}
		const confirmed = window.confirm(
			"Power down RTCore-controlled drives now? This will disable and disarm the configured axes.",
		);
		if (!confirmed) {
			return;
		}
		setPendingJointAction("power-down");
		setCommissioningStatus({
			tone: "info",
			message: "Requesting drive power-down...",
		});
		try {
			const result = await post("/control/power-down", { wait_for_idle: true }) as MotionStatusResponse | null;
			if (result) {
				setMotionStatus(result);
				const refreshed = await refreshJointAngles(false);
				setCommissioningStatus(
					refreshed
						? {
							tone: "success",
							message: "Drive power-down requested with safe stop/disarm sequencing. Live joint feedback refreshed.",
						}
						: {
							tone: "success",
							message: "Drive power-down requested with safe stop/disarm sequencing. Refresh joint feedback after the axes disarm.",
						},
				);
			} else {
				setCommissioningStatus({
					tone: "error",
					message: "Failed to request drive power-down.",
				});
			}
		} finally {
			setPendingJointAction(null);
		}
	}, [pendingJointAction, post, refreshJointAngles, setMotionStatus]);

	const captureEncoderRetentionSnapshot = useCallback(async (phase: "before_power_down" | "after_power_up") => {
		if (pendingJointAction) {
			return;
		}
		setPendingJointAction(`encoder-retention-${phase}`);
		setCommissioningStatus({
			tone: "info",
			message: phase === "before_power_down"
				? "Capturing encoder retention snapshot before drive power-down..."
				: "Capturing encoder retention snapshot after drive power-up...",
		});
		try {
			const result = await post("/control/encoder-retention/capture", {
				phase,
				experiment_id: encoderRetentionExperimentId ?? undefined,
			}) as Record<string, unknown> | null;
			const nextExperimentId = typeof result?.experiment_id === "string" ? result.experiment_id : null;
			if (nextExperimentId) {
				setEncoderRetentionExperimentId(nextExperimentId);
			}
			const comparison = result?.comparison as Record<string, unknown> | undefined;
			setCommissioningStatus({
				tone: "success",
				message: comparison
					? `Captured ${phase.replaceAll("_", " ")} snapshot for experiment ${nextExperimentId ?? "unknown"} and wrote a before/after comparison artifact.`
					: `Captured ${phase.replaceAll("_", " ")} snapshot for experiment ${nextExperimentId ?? "unknown"}.`,
			});
		} catch (error) {
			setCommissioningStatus({
				tone: "error",
				message: error instanceof Error
					? error.message
					: `Failed to capture ${phase.replaceAll("_", " ")} retention snapshot.`,
			});
		} finally {
			setPendingJointAction(null);
		}
	}, [pendingJointAction, post, encoderRetentionExperimentId]);

	// ------------------------
	// Incremental jog helpers
	// ------------------------
	const performIncrementalLinearJog = useCallback(
		async (axis: "x" | "y" | "z", direction: 1 | -1) => {
			await runDiscreteMotionCommand(`move-line-${axis}-${direction}`, async () => {
				// Use the user-configured linear base value from the UI as the incremental step size.
				// The field is labeled in mm/s for realtime jog, but here we treat the numeric value
				// as a distance in millimeters for a single incremental move.
				const stepMeters = (linBaseMmS / 1000.0) * direction;
				const dx = axis === "x" ? stepMeters : 0;
				const dy = axis === "y" ? stepMeters : 0;
				const dz = axis === "z" ? stepMeters : 0;
				const result = await post("/control/move-line-relative", {
					dx,
					dy,
					dz,
					speed_multiplier: speedMult,
					closed: false,
				}) as MotionStatusResponse | null;
				if (result) {
					setMotionStatus(result);
				}
			});
		},
		[linBaseMmS, post, runDiscreteMotionCommand, speedMult],
	);

	const performIncrementalAngularJog = useCallback(
		async (axis: "roll" | "pitch" | "yaw", direction: 1 | -1) => {
			await runDiscreteMotionCommand(`rotate-${axis}-${direction}`, async () => {
				// Use the user-configured angular base value from the UI as the incremental step size.
				const angleDeg = angBaseDegS * direction;
				const commandedAngularDegS = Math.max(1e-3, effectiveAngularDegS);
				const result = await post("/control/rotate", {
					axis,
					angle_deg: angleDeg,
					duration_s: Math.abs(angleDeg) / commandedAngularDegS,
				}) as MotionStatusResponse | null;
				if (result) {
					setMotionStatus(result);
				}
			});
		},
		[angBaseDegS, effectiveAngularDegS, post, runDiscreteMotionCommand],
	);

	const changeLinearCount = useCallback(async (
		axis: "x" | "y" | "z",
		delta: number,
		options?: { releaseWhenZero?: boolean; stopReason?: string },
	) => {
		linCountsRef.current = { ...linCountsRef.current, [axis]: linCountsRef.current[axis] + delta };
		if (!hasActiveJogInput()) {
			if (options?.releaseWhenZero === false) {
				await enqueueJogState(buildJogStatePayload([0, 0, 0, 0, 0, 0], {
					active: true,
					stop_reason: options.stopReason ?? "pointer-cancel-hold-zero",
				}), true);
				return;
			}
			await releaseJogHold(options?.stopReason);
			return;
		}
		jogHoldActiveRef.current = true;
		ensureJogPolling();
		await sendJogTick();
	}, [buildJogStatePayload, enqueueJogState, ensureJogPolling, hasActiveJogInput, releaseJogHold, sendJogTick]);

	const changeAngularCount = useCallback(async (
		axis: "x" | "y" | "z",
		delta: number,
		options?: { releaseWhenZero?: boolean; stopReason?: string },
	) => {
		angCountsRef.current = { ...angCountsRef.current, [axis]: angCountsRef.current[axis] + delta };
		if (!hasActiveJogInput()) {
			if (options?.releaseWhenZero === false) {
				await enqueueJogState(buildJogStatePayload([0, 0, 0, 0, 0, 0], {
					active: true,
					stop_reason: options.stopReason ?? "pointer-cancel-hold-zero",
				}), true);
				return;
			}
			await releaseJogHold(options?.stopReason);
			return;
		}
		jogHoldActiveRef.current = true;
		ensureJogPolling();
		await sendJogTick();
	}, [buildJogStatePayload, enqueueJogState, ensureJogPolling, hasActiveJogInput, releaseJogHold, sendJogTick]);

	const onPress = useCallback((fn: () => void) => (e: React.PointerEvent<HTMLButtonElement>) => {
		(e.currentTarget as HTMLButtonElement).setPointerCapture(e.pointerId);
		fn();
	}, []);
	const onRelease = useCallback((fn: () => void) => (e: React.PointerEvent<HTMLButtonElement>) => {
		try {
			(e.currentTarget as HTMLButtonElement).releasePointerCapture(e.pointerId);
		} catch {}
		if (ignoreNextJogReleaseRef.current) {
			ignoreNextJogReleaseRef.current = false;
			return;
		}
		fn();
	}, []);
	const onCancel = useCallback((fn: () => void) => (e: React.PointerEvent<HTMLButtonElement>) => {
		try {
			(e.currentTarget as HTMLButtonElement).releasePointerCapture(e.pointerId);
		} catch {}
		ignoreNextJogReleaseRef.current = true;
		window.setTimeout(() => {
			ignoreNextJogReleaseRef.current = false;
		}, 750);
		fn();
	}, []);

	const controlsAreCollapsible = splitStatusSections && typeof onToggleControlsCollapsed === "function";
	const drivePowerSection = (
		<div className={`${splitStatusSections ? "" : "mb-3 "}border border-cyan-500/25 bg-cyan-500/5 p-3`}>
			<div className="mb-2 flex items-center justify-between gap-2 text-xs text-cyan-100/90">
				<span className="font-semibold uppercase tracking-[0.18em]">Drive Power</span>
				<span
					className={`rounded border px-2 py-0.5 text-[10px] font-semibold uppercase tracking-[0.16em] ${
						isDrivePowerActive
							? "border-emerald-400/40 bg-emerald-400/10 text-emerald-100"
							: "border-amber-400/40 bg-amber-400/10 text-amber-100"
					}`}
				>
					{driveFaults?.driver_state ?? (requiresExplicitDrivePower ? "STATUS PENDING" : "NOT APPLICABLE")}
				</span>
			</div>
			<div className="mb-2 text-[11px] leading-relaxed text-slate-300">
				{requiresExplicitDrivePower
					? "Startup leaves the RTCore drives disarmed. Power them up explicitly when you are ready to energize the axes."
					: "Drive power controls are only used for the EtherCAT RTCore backend."}
			</div>
			<div className="mb-3 text-[10px] text-slate-400">
				{driveFaults
					? `Backend ${driveFaults.servo_backend ?? "unknown"} | driver ${driveFaults.driver_state ?? "unknown"} | request ${drivePowerRequested ? "enable" : "none"} | EtherCAT ${driveFaults.ethercat_master_state ?? "unknown"} | RTCore ${driveFaults.rtcore_state ?? "unknown"} | enable mask ${driveFaults.axis_enable_mask_hex ?? "unknown"} | op-enabled ${actualOpEnabledAxes}/${totalDriveAxes} | statusword feedback ${statuswordFeedbackAxes}/${totalDriveAxes}`
					: requiresExplicitDrivePower
						? "Waiting for live drive-state telemetry..."
						: "No RTCore drive state available for this backend."}
			</div>
			<div className="grid grid-cols-3 gap-2">
				<button
					type="button"
					className={`rounded border px-2 py-2 text-[11px] font-semibold transition disabled:cursor-not-allowed disabled:opacity-50 ${
						isDrivePowerActive
							? "border-emerald-500/30 bg-emerald-400/10 text-emerald-100 hover:border-emerald-300/60 hover:bg-emerald-300/15"
							: "border-cyan-500/30 bg-cyan-400/10 text-cyan-100 hover:border-cyan-300/60 hover:bg-cyan-300/15"
					}`}
					onClick={() => {
						void handlePowerUpDrives();
					}}
					disabled={pendingJointAction !== null || !requiresExplicitDrivePower || isDrivePowerActive || drivePowerRequested || powerUpBlocked}
					title={
						drivePowerRequested && !isDrivePowerActive
							? "Enable already requested; waiting for actual drive feedback."
							: powerUpBlocked
								? powerUpBlockerMessage
								: "Explicitly arm and enable RTCore-controlled axes"
					}
				>
					{pendingJointAction === "power-up" ? "..." : "Power Up Drives"}
				</button>
				<button
					type="button"
					className="rounded border border-rose-500/35 bg-rose-400/10 px-2 py-2 text-[11px] font-semibold text-rose-100 transition hover:border-rose-300/60 hover:bg-rose-300/15 disabled:cursor-not-allowed disabled:opacity-50"
					onClick={() => {
						void handlePowerDownDrives();
					}}
					disabled={pendingJointAction !== null || !requiresExplicitDrivePower || (!isDrivePowerActive && !drivePowerRequested)}
					title="Explicitly disable and disarm RTCore-controlled axes"
				>
					{pendingJointAction === "power-down" ? "..." : "Power Down Drives"}
				</button>
				<button
					type="button"
					className="rounded border border-amber-500/40 bg-amber-400/10 px-2 py-2 text-[11px] font-semibold text-amber-100 transition hover:border-amber-300/60 hover:bg-amber-300/15 disabled:cursor-not-allowed disabled:opacity-50"
					onClick={() => {
						void handleResetFaults();
					}}
					disabled={pendingJointAction !== null || !requiresExplicitDrivePower}
					title="Request a DS402 fault reset for all RTCore-controlled axes"
				>
					{pendingJointAction?.startsWith("reset-faults") ? "..." : "Reset All Faults"}
				</button>
			</div>
			<div className="mt-3 grid grid-cols-2 gap-2">
				<button
					type="button"
					className="rounded border border-slate-600/70 bg-slate-900/70 px-2 py-2 text-[11px] font-semibold text-slate-100 transition hover:border-slate-400/70 hover:bg-slate-800 disabled:cursor-not-allowed disabled:opacity-50"
					onClick={() => {
						void captureEncoderRetentionSnapshot("before_power_down");
					}}
					disabled={pendingJointAction !== null}
					title="Log the current encoder and drive state before powering the drives down"
				>
					{pendingJointAction === "encoder-retention-before_power_down" ? "..." : "Log Before Power Down"}
				</button>
				<button
					type="button"
					className="rounded border border-slate-600/70 bg-slate-900/70 px-2 py-2 text-[11px] font-semibold text-slate-100 transition hover:border-slate-400/70 hover:bg-slate-800 disabled:cursor-not-allowed disabled:opacity-50"
					onClick={() => {
						void captureEncoderRetentionSnapshot("after_power_up");
					}}
					disabled={pendingJointAction !== null}
					title="Log the current encoder and drive state after powering the drives back up"
				>
					{pendingJointAction === "encoder-retention-after_power_up" ? "..." : "Log After Power Up"}
				</button>
			</div>
			<div className="mt-2 text-[10px] text-slate-400">
				{encoderRetentionExperimentId
					? `Encoder retention experiment: ${encoderRetentionExperimentId}`
					: "Encoder retention logging will create an experiment id on the first capture."}
			</div>
			{driveFaults ? (
				<div
					className={`mt-3 rounded border px-2 py-2 text-[11px] leading-relaxed ${
						activeDriveFaultAxes.length > 0
							? "border-rose-500/30 bg-rose-400/10 text-rose-100"
							: "border-slate-700/60 bg-slate-900/70 text-slate-200"
					}`}
				>
					<div className="flex items-center justify-between gap-2">
						<span className="font-semibold">
							{activeDriveFaultAxes.length > 0 ? `Drive faults detected (${activeDriveFaultAxes.length})` : "Drive fault status"}
						</span>
						<span
							className={`rounded border px-2 py-0.5 text-[10px] font-semibold uppercase tracking-[0.16em] ${
								activeDriveFaultAxes.length > 0
									? "border-rose-400/40 bg-rose-400/10 text-rose-100"
									: "border-slate-600/70 bg-slate-800/70 text-slate-300"
							}`}
						>
							{driveFaults.physical_state ?? "unknown"}
						</span>
					</div>
					<div className="mt-1 text-[10px] text-current/70">
						Fault reference:{" "}
						{driveFaults.reference?.available
							? driveFaults.reference.label ?? driveFaults.reference.profile_id ?? "configured"
							: "raw only"}
					</div>
					{activeDriveFaultAxes.length > 0 ? (
						<div className="mt-1 text-[10px] text-current/70">
							Reset individual faulted joints below, or use <span className="font-semibold">Reset All Faults</span> to
							pulse DS402 fault reset across every RTCore axis.
						</div>
					) : null}
					{activeDriveFaultAxes.length > 0 ? (
						<div className="mt-2 space-y-1">
							{activeDriveFaultAxes.map((axis) => (
								<div
									key={`drive-fault-${axis.axis}`}
									className="rounded border border-current/15 bg-black/10 px-2 py-1 text-[10px]"
								>
									<div className="flex items-center justify-between gap-2">
										<span className="min-w-0 flex-1">{formatDriveFaultDescription(axis)}</span>
										{axis.fault?.resettable === true &&
										typeof axis.logical_joint === "number" &&
										Number.isFinite(axis.logical_joint) ? (
											<button
												type="button"
												className="shrink-0 rounded border border-amber-400/35 bg-amber-400/10 px-2 py-0.5 text-[10px] font-semibold uppercase tracking-[0.12em] text-amber-100 transition hover:border-amber-300/60 hover:bg-amber-300/15 disabled:cursor-not-allowed disabled:opacity-50"
												onClick={() => {
													void handleResetFaults(axis.logical_joint);
												}}
												disabled={pendingJointAction !== null}
												title={`Request a DS402 fault reset for J${axis.logical_joint}`}
											>
												Reset J{axis.logical_joint}
											</button>
										) : null}
									</div>
								</div>
							))}
						</div>
					) : (
						<div className="mt-2 text-[10px] text-current/70">
							No active drive faults in the latest telemetry snapshot.
						</div>
					)}
				</div>
			) : null}
		</div>
	);
	const motionStateSection = (
		<div className={`${splitStatusSections ? "" : "mb-3 "}border border-slate-700/60 bg-slate-950/30 p-3`}>
			<div className="mb-2 flex items-center justify-between gap-2 text-xs text-slate-300/80">
				<span className="font-semibold uppercase tracking-[0.18em]">Motion State</span>
				<span
					className={`rounded border px-2 py-0.5 text-[10px] font-semibold uppercase tracking-[0.16em] ${
						motionStateName === "faulted" || motionStateName === "aborted" || motionStateName === "underrun"
							? "border-rose-500/40 bg-rose-400/10 text-rose-100"
							: motionBusy
								? "border-cyan-500/40 bg-cyan-400/10 text-cyan-100"
								: motionStateName === "completed"
									? "border-emerald-500/40 bg-emerald-400/10 text-emerald-100"
									: "border-slate-600/70 bg-slate-800/70 text-slate-300"
					}`}
				>
					{formatMotionStateLabel(motionStateName)}
				</span>
			</div>
			<div className="text-[11px] leading-relaxed text-slate-300">
				{motionStatus
					? `Source ${motionStatus.source_of_truth ?? "controller"} | scope ${motionStatus.completion_scope ?? "unknown"}`
					: "Waiting for controller motion status..."}
			</div>
			<div className="mt-1 text-[10px] text-slate-400">
				{motionStatus
					? `traj ${motionTrajectoryId ?? "none"} | mode ${motionStatus.execution?.active_mode_name ?? "n/a"} | queue ${motionStatus.execution?.queue_depth ?? 0}/${motionStatus.execution?.queue_capacity ?? 0} | done ${motionStatus.execution?.motion_done ? "yes" : "no"} | stale ${motionStatus.execution?.stale_command ? "yes" : "no"} | underruns ${motionStatus.execution?.underrun_count ?? 0}`
					: "No RTCore execution metadata yet."}
			</div>
			<div
				className={`mt-2 rounded border px-2 py-2 text-[10px] leading-relaxed ${
					powerTransitionKnown && powerTransitionSafe
						? "border-emerald-500/30 bg-emerald-400/10 text-emerald-100"
						: "border-amber-500/30 bg-amber-400/10 text-amber-100"
				}`}
			>
				<div className="font-semibold uppercase tracking-[0.14em]">
					{powerTransitionKnown && powerTransitionSafe
						? "Safe For Power Transition"
						: "Power-Up Blocked"}
				</div>
				<div className="mt-1">
					{powerTransitionKnown && powerTransitionSafe
						? "Runtime is neutral and synchronized; explicit drive enable is allowed."
						: powerUpBlockerMessage}
				</div>
			</div>
		</div>
	);
	const controlsBody = (
		<>
			{/* Removed step move blocks; unified under realtime jog below */}
			{showGripperPanel ? (
				<div className="mb-3 rounded-lg border border-slate-700/60 p-2">
					<div className="mb-2 flex items-center justify-between text-xs text-slate-300/80">
						<span className="font-semibold">Gripper</span>
						<span className="tabular-nums">{grip}°</span>
					</div>
					<input
						type="range"
						min={0}
						max={180}
						value={grip}
						onChange={(e) => handleGripChange(Number(e.target.value))}
						className="w-full accent-cyan-400"
					/>
					<div className="mt-1 flex gap-2">
						<button className="rounded bg-slate-800 px-2 py-1 hover:bg-slate-700" onClick={() => handleGripChange(120)}>Open</button>
						<button className="rounded bg-slate-800 px-2 py-1 hover:bg-slate-700" onClick={() => handleGripChange(0)}>Close</button>
					</div>
				</div>
			) : null}
			<div className="mb-3 rounded-lg border border-slate-700/60 p-2">
				<div className="mb-2 flex items-center justify-between text-xs text-slate-300/80">
					<span className="font-semibold">Speed Multiplier</span>
					<span className="tabular-nums">{speedMult.toFixed(2)}x</span>
				</div>
				<input
					type="range"
					min={0}
					max={1000}
					value={speedVal}
					onChange={(e) => setSpeedVal(Number(e.target.value))}
					className="w-full accent-cyan-400"
				/>
			</div>
			<div className="mb-3 rounded-lg border border-slate-700/60 p-2">
				<div className="mb-2 flex items-center justify-between text-xs text-slate-300/80">
					<span className="font-semibold">Realtime Jog</span>
					<div className="flex items-center gap-2">
						<label className="flex items-center gap-1 text-[12px]">
							<input type="checkbox" checked={deadman} onChange={(e) => onDeadmanToggle(e.target.checked)} />
							Deadman
						</label>
						<label className="flex items-center gap-1 text-[12px]">
							<input type="checkbox" onChange={(e) => onDebugToggle(e.target.checked)} />
							Debug
						</label>
						<button
							className={`rounded px-2 py-1 ${jogEnabled ? "bg-rose-600 text-white" : "bg-slate-800 hover:bg-slate-700"}`}
							onClick={async () => (jogEnabled ? await stopJog() : armJogMode())}
						>
							{jogEnabled ? "Disarm Jog" : "Arm Jog"}
						</button>
					</div>
				</div>
				<div className="mb-2 grid grid-cols-2 gap-2">
					<label className="flex items-center justify-between rounded border border-slate-700/60 bg-slate-800/60 px-2 py-1 text-xs">
						<span>Linear Base (mm/s)</span>
						<input
							className="w-16 rounded bg-slate-900/60 px-1 text-right outline-none"
							type="number"
							value={linBaseMmS}
							min={0}
							max={1000}
							onChange={(e) => setLinBaseMmS(Number(e.target.value))}
						/>
					</label>
					<label className="flex items-center justify-between rounded border border-slate-700/60 bg-slate-800/60 px-2 py-1 text-xs">
						<span>Angular Base (deg/s)</span>
						<input
							className="w-16 rounded bg-slate-900/60 px-1 text-right outline-none"
							type="number"
							value={angBaseDegS}
							min={0}
							max={360}
							onChange={(e) => setAngBaseDegS(Number(e.target.value))}
						/>
					</label>
				</div>
				<div className="mb-2 rounded border border-slate-700/60 bg-slate-950/40 px-2 py-2 text-[11px] text-slate-300/90">
					<div className="mb-1 flex items-center justify-between">
						<span className="font-semibold text-slate-200">Backend Jog Command (live)</span>
						<span className="tabular-nums text-cyan-200/80">
							base: {effectiveLinearMS.toFixed(4)} m/s, {effectiveAngularDegS.toFixed(2)} deg/s
						</span>
					</div>
					<div className="tabular-nums">
						<span className="text-slate-400">linear m/s:</span>{" "}
						vx={lastJogCommand[0].toFixed(6)} vy={lastJogCommand[1].toFixed(6)} vz={lastJogCommand[2].toFixed(6)}
					</div>
					<div className="tabular-nums">
						<span className="text-slate-400">angular deg/s:</span>{" "}
						roll={lastJogCommand[3].toFixed(3)} pitch={lastJogCommand[4].toFixed(3)} yaw={lastJogCommand[5].toFixed(3)}
					</div>
					<div className="mt-1 text-[10px] text-slate-500">
						Jog mode enables hold-to-jog. The controller session only exists while a jog button is actively held.
					</div>
				</div>
				<div className="grid grid-cols-3 gap-1" style={{ touchAction: "none" }}>
					<button
						className="rounded bg-slate-800 px-2 py-1 hover:bg-slate-700"
						onPointerDown={onPress(() => {
							if (jogEnabled) {
								changeLinearCount("x", +1).catch(() => {});
							}
						})}
						onPointerUp={onRelease(() => {
							if (jogEnabled) {
								changeLinearCount("x", -1).catch(() => {});
							} else {
								performIncrementalLinearJog("x", +1).catch(() => {});
							}
						})}
						onPointerCancel={onCancel(() => {
							if (jogEnabled) {
								changeLinearCount("x", -1, { releaseWhenZero: false }).catch(() => {});
							}
						})}
					>
						+X
					</button>
					<button
						className="rounded bg-slate-800 px-2 py-1 hover:bg-slate-700"
						onPointerDown={onPress(() => {
							if (jogEnabled) {
								changeLinearCount("y", +1).catch(() => {});
							}
						})}
						onPointerUp={onRelease(() => {
							if (jogEnabled) {
								changeLinearCount("y", -1).catch(() => {});
							} else {
								performIncrementalLinearJog("y", +1).catch(() => {});
							}
						})}
						onPointerCancel={onCancel(() => {
							if (jogEnabled) {
								changeLinearCount("y", -1, { releaseWhenZero: false }).catch(() => {});
							}
						})}
					>
						+Y
					</button>
					<button
						className="rounded bg-slate-800 px-2 py-1 hover:bg-slate-700"
						onPointerDown={onPress(() => {
							if (jogEnabled) {
								changeLinearCount("z", +1).catch(() => {});
							}
						})}
						onPointerUp={onRelease(() => {
							if (jogEnabled) {
								changeLinearCount("z", -1).catch(() => {});
							} else {
								performIncrementalLinearJog("z", +1).catch(() => {});
							}
						})}
						onPointerCancel={onCancel(() => {
							if (jogEnabled) {
								changeLinearCount("z", -1, { releaseWhenZero: false }).catch(() => {});
							}
						})}
					>
						+Z
					</button>
					<button
						className="rounded bg-slate-800 px-2 py-1 hover:bg-slate-700"
						onPointerDown={onPress(() => {
							if (jogEnabled) {
								changeLinearCount("x", -1).catch(() => {});
							}
						})}
						onPointerUp={onRelease(() => {
							if (jogEnabled) {
								changeLinearCount("x", +1).catch(() => {});
							} else {
								performIncrementalLinearJog("x", -1).catch(() => {});
							}
						})}
						onPointerCancel={onCancel(() => {
							if (jogEnabled) {
								changeLinearCount("x", +1, { releaseWhenZero: false }).catch(() => {});
							}
						})}
					>
						-X
					</button>
					<button
						className="rounded bg-slate-800 px-2 py-1 hover:bg-slate-700"
						onPointerDown={onPress(() => {
							if (jogEnabled) {
								changeLinearCount("y", -1).catch(() => {});
							}
						})}
						onPointerUp={onRelease(() => {
							if (jogEnabled) {
								changeLinearCount("y", +1).catch(() => {});
							} else {
								performIncrementalLinearJog("y", -1).catch(() => {});
							}
						})}
						onPointerCancel={onCancel(() => {
							if (jogEnabled) {
								changeLinearCount("y", +1, { releaseWhenZero: false }).catch(() => {});
							}
						})}
					>
						-Y
					</button>
					<button
						className="rounded bg-slate-800 px-2 py-1 hover:bg-slate-700"
						onPointerDown={onPress(() => {
							if (jogEnabled) {
								changeLinearCount("z", -1).catch(() => {});
							}
						})}
						onPointerUp={onRelease(() => {
							if (jogEnabled) {
								changeLinearCount("z", +1).catch(() => {});
							} else {
								performIncrementalLinearJog("z", -1).catch(() => {});
							}
						})}
						onPointerCancel={onCancel(() => {
							if (jogEnabled) {
								changeLinearCount("z", +1, { releaseWhenZero: false }).catch(() => {});
							}
						})}
					>
						-Z
					</button>
				</div>
				<div className="mt-2 grid grid-cols-3 gap-1" style={{ touchAction: "none" }}>
					<button
						className="rounded bg-slate-800 px-2 py-1 hover:bg-slate-700"
						onPointerDown={onPress(() => {
							if (jogEnabled) {
								changeAngularCount("x", +1).catch(() => {});
							}
						})}
						onPointerUp={onRelease(() => {
							if (jogEnabled) {
								changeAngularCount("x", -1).catch(() => {});
							} else {
								performIncrementalAngularJog("roll", +1).catch(() => {});
							}
						})}
						onPointerCancel={onCancel(() => {
							if (jogEnabled) {
								changeAngularCount("x", -1, { releaseWhenZero: false }).catch(() => {});
							}
						})}
					>
						+Roll
					</button>
					<button
						className="rounded bg-slate-800 px-2 py-1 hover:bg-slate-700"
						onPointerDown={onPress(() => {
							if (jogEnabled) {
								changeAngularCount("y", +1).catch(() => {});
							}
						})}
						onPointerUp={onRelease(() => {
							if (jogEnabled) {
								changeAngularCount("y", -1).catch(() => {});
							} else {
								performIncrementalAngularJog("pitch", +1).catch(() => {});
							}
						})}
						onPointerCancel={onCancel(() => {
							if (jogEnabled) {
								changeAngularCount("y", -1, { releaseWhenZero: false }).catch(() => {});
							}
						})}
					>
						+Pitch
					</button>
					<button
						className="rounded bg-slate-800 px-2 py-1 hover:bg-slate-700"
						onPointerDown={onPress(() => {
							if (jogEnabled) {
								changeAngularCount("z", +1).catch(() => {});
							}
						})}
						onPointerUp={onRelease(() => {
							if (jogEnabled) {
								changeAngularCount("z", -1).catch(() => {});
							} else {
								performIncrementalAngularJog("yaw", +1).catch(() => {});
							}
						})}
						onPointerCancel={onCancel(() => {
							if (jogEnabled) {
								changeAngularCount("z", -1, { releaseWhenZero: false }).catch(() => {});
							}
						})}
					>
						+Yaw
					</button>
					<button
						className="rounded bg-slate-800 px-2 py-1 hover:bg-slate-700"
						onPointerDown={onPress(() => {
							if (jogEnabled) {
								changeAngularCount("x", -1).catch(() => {});
							}
						})}
						onPointerUp={onRelease(() => {
							if (jogEnabled) {
								changeAngularCount("x", +1).catch(() => {});
							} else {
								performIncrementalAngularJog("roll", -1).catch(() => {});
							}
						})}
						onPointerCancel={onCancel(() => {
							if (jogEnabled) {
								changeAngularCount("x", +1, { releaseWhenZero: false }).catch(() => {});
							}
						})}
					>
						-Roll
					</button>
					<button
						className="rounded bg-slate-800 px-2 py-1 hover:bg-slate-700"
						onPointerDown={onPress(() => {
							if (jogEnabled) {
								changeAngularCount("y", -1).catch(() => {});
							}
						})}
						onPointerUp={onRelease(() => {
							if (jogEnabled) {
								changeAngularCount("y", +1).catch(() => {});
							} else {
								performIncrementalAngularJog("pitch", -1).catch(() => {});
							}
						})}
						onPointerCancel={onCancel(() => {
							if (jogEnabled) {
								changeAngularCount("y", +1, { releaseWhenZero: false }).catch(() => {});
							}
						})}
					>
						-Pitch
					</button>
					<button
						className="rounded bg-slate-800 px-2 py-1 hover:bg-slate-700"
						onPointerDown={onPress(() => {
							if (jogEnabled) {
								changeAngularCount("z", -1).catch(() => {});
							}
						})}
						onPointerUp={onRelease(() => {
							if (jogEnabled) {
								changeAngularCount("z", +1).catch(() => {});
							} else {
								performIncrementalAngularJog("yaw", -1).catch(() => {});
							}
						})}
						onPointerCancel={onCancel(() => {
							if (jogEnabled) {
								changeAngularCount("z", +1, { releaseWhenZero: false }).catch(() => {});
							}
						})}
					>
						-Yaw
					</button>
				</div>
			</div>
			<div className="mb-3 rounded-lg border border-slate-700/60 bg-slate-950/30 p-2">
				<div className="mb-2 flex items-center justify-between gap-2 text-xs text-slate-300/80">
					<span className="font-semibold">Joint Commissioning</span>
					<div className="flex items-center gap-2">
						<span className="rounded border border-amber-500/30 bg-amber-400/10 px-2 py-0.5 text-[10px] font-semibold uppercase tracking-[0.18em] text-amber-200/90">
							RTCore cap 100 RPM
						</span>
						<button
							type="button"
							className="rounded border border-slate-700/70 bg-slate-900/60 px-2 py-1 text-[10px] font-semibold uppercase tracking-[0.16em] text-slate-300 transition hover:border-slate-500/80 hover:text-white"
							onClick={() => {
								setCommissioningExpanded((expanded) => !expanded);
							}}
							aria-expanded={commissioningExpanded}
						>
							{commissioningExpanded ? "Hide" : "Show"}
						</button>
					</div>
				</div>
				{commissioningExpanded ? (
					<>
						<div className="mb-2 text-[11px] leading-relaxed text-slate-400">
							Use small relative steps while aligning joints for zero capture. This panel assumes the RTCore setup-time
							safety clamp remains at <code className="rounded bg-slate-900/80 px-1 py-0.5 text-slate-200">--max-rpm 100</code>.
						</div>
						<div className="mb-2 rounded border border-amber-500/20 bg-amber-400/5 px-2 py-2 text-[11px] leading-relaxed text-amber-100/90">
							Zero capture writes a persistent logical offset for the selected joint. Refresh feedback first, jog the joint
							into alignment, then capture zero only when the live angle shown for that joint matches what you expect.
						</div>
						{requiresExplicitDrivePower && !drivePowerReady ? (
							<div className="mb-2 rounded border border-cyan-500/20 bg-cyan-400/10 px-2 py-1.5 text-[11px] text-cyan-100">
								Drives are currently disarmed. Use the <span className="font-semibold">{driveControlsReferenceLabel}</span> before jogging.
							</div>
						) : null}
						<div className="mb-2">
							<div className="mb-1 flex items-center justify-between gap-2">
								<span className="text-[11px] uppercase tracking-[0.18em] text-slate-500">Joint Step</span>
								<div className="flex items-center gap-2">
									<button
										type="button"
										className="rounded border border-slate-700/70 bg-slate-900/60 px-2 py-1 text-[11px] font-medium text-slate-200 transition hover:border-slate-500/80 hover:text-white"
										onClick={() => {
											void handleRefreshJointFeedback();
										}}
									>
										Refresh
									</button>
									<div className="flex items-center gap-1">
										{JOINT_STEP_OPTIONS_DEG.map((value) => (
											<button
												key={value}
												type="button"
												className={`rounded border px-2 py-1 text-[11px] font-medium tabular-nums transition ${
													jointStepDeg === value
														? "border-cyan-400/70 bg-cyan-400/15 text-cyan-100"
														: "border-slate-700/70 bg-slate-900/60 text-slate-300 hover:border-slate-500/80 hover:text-slate-100"
												}`}
												onClick={() => setJointStepDeg(value)}
											>
												{value}°
											</button>
										))}
									</div>
								</div>
							</div>
							<div className="flex justify-end">
								<div className="flex w-full max-w-[15rem] items-center gap-1 rounded border border-slate-700/70 bg-slate-900/60 px-1.5 py-1">
									<span className="text-[10px] font-semibold uppercase tracking-[0.16em] text-slate-500">
										Custom
									</span>
									<input
										type="number"
										inputMode="decimal"
										step="0.001"
										placeholder="+/- deg"
										value={customJointStepInput}
										onChange={(event) => {
											setCustomJointStepInput(event.target.value);
										}}
										onBlur={() => {
											if (customJointStepInput.trim()) {
												applyCustomJointStep();
											}
										}}
										onKeyDown={(event) => {
											if (event.key === "Enter") {
												event.preventDefault();
												applyCustomJointStep();
											}
										}}
										className="min-w-0 flex-1 rounded border border-slate-700/70 bg-slate-950/60 px-2 py-1 text-[11px] font-medium tabular-nums text-slate-100 outline-none transition placeholder:text-slate-500 focus:border-cyan-400/70 focus:ring-1 focus:ring-cyan-400/40"
										title="Enter a signed or unsigned custom joint step in degrees. The +/- jog buttons apply direction."
									/>
									<button
										type="button"
										className="rounded border border-cyan-500/40 bg-cyan-400/10 px-2 py-1 text-[11px] font-semibold text-cyan-100 transition hover:border-cyan-300/60 hover:bg-cyan-300/15"
										onClick={() => {
											applyCustomJointStep();
										}}
										title="Use the custom joint step"
									>
										Use
									</button>
								</div>
							</div>
						</div>
						<div className="mb-2 space-y-1" aria-live="polite">
							{Array.from({ length: COMMISSIONING_MESSAGE_SLOT_COUNT }, (_, index) => {
								const entry = commissioningMessages[index] ?? null;
								return (
									<div
										key={entry?.key ?? `commissioning-message-slot-${index}`}
										data-testid="commissioning-message-slot"
										aria-hidden={entry ? undefined : true}
										className={`h-9 overflow-hidden rounded border px-2 py-0.5 text-[11px] leading-4 ${
											commissioningMessageToneClasses(entry?.tone ?? "neutral")
										} ${entry ? "" : "invisible"}`}
									>
										<span
											style={{
												display: "-webkit-box",
												WebkitLineClamp: 2,
												WebkitBoxOrient: "vertical",
												overflow: "hidden",
											}}
										>
											{entry?.message ?? "\u00A0"}
										</span>
									</div>
								);
							})}
						</div>
						<div className="space-y-1">
							{Array.from({ length: jointAnglesDeg.length > 0 ? jointAnglesDeg.length : 6 }, (_, jointIndex) => {
								const jointNumber = jointIndex + 1;
								const angleDeg = jointAnglesDeg[jointIndex];
								const angleLabel = Number.isFinite(angleDeg) ? `${angleDeg.toFixed(2)}°` : "--";
								const staleTelemetryMessage = telemetryStale && Number.isFinite(angleDeg)
									? `display holding last good sample${telemetryStaleAge !== null ? ` (${telemetryStaleAge.toFixed(1)}s)` : ""}`
									: null;
								const driveAxis = commissioningDriveAxesByJoint.get(jointNumber);
								const hasDriveAxisFeedback = typeof driveAxis?.statusword === "number" && Number.isFinite(driveAxis.statusword) && driveAxis.statusword !== 0;
								const nativeHomeStatus = formatNativeHomeStatus(driveAxis, driveFaults);
								const driveNativeTruthStatus = formatDriveNativeTruthStatus(driveAxis);
								const nativeHomeActiveForJoint = Boolean(driveAxis?.native_home_active)
									|| (driveAxis ? ((nativeHomeActiveAxisMask & (1 << driveAxis.axis)) !== 0) : false);
								const isPending = pendingJointAction === `jog-${jointIndex}`
									|| pendingJointAction === `zero-${jointIndex}`
									|| pendingJointAction === `native-home-${jointIndex}`;
								const hasLiveFeedback = Number.isFinite(angleDeg);
								const hasDisplayOrStaleFeedback = hasLiveFeedback || telemetryStale;
								const controlsDisabled = pendingJointAction !== null || motionBusy;
								const jogDisabled = controlsDisabled || !hasDisplayOrStaleFeedback || !drivePowerReady;
								const zeroDisabled = controlsDisabled || !hasLiveFeedback;
								const nativeHomeDisabled = controlsDisabled || nativeHomeBusy || (!hasLiveFeedback && !hasDriveAxisFeedback);
								return (
									<div
										key={jointNumber}
										className="flex items-center gap-2 rounded border border-slate-700/60 bg-slate-900/45 px-2 py-1.5"
									>
										<div className="min-w-[3.25rem]">
											<div className="text-[11px] font-semibold uppercase tracking-[0.18em] text-slate-400">
												J{jointNumber}
											</div>
											<div className="tabular-nums text-sm font-semibold text-cyan-100">{angleLabel}</div>
											{staleTelemetryMessage ? (
												<div className="mt-0.5 max-w-[14rem] text-[10px] leading-snug text-amber-200/90">
													{staleTelemetryMessage}
												</div>
											) : null}
											{nativeHomeStatus ? (
												<div className="mt-0.5 max-w-[12rem] text-[10px] leading-snug text-amber-200/90">
													{nativeHomeStatus}
												</div>
											) : null}
											{driveNativeTruthStatus ? (
												<div
													className={`mt-0.5 max-w-[14rem] text-[10px] leading-snug ${
														driveNativeTruthStatus.tone === "valid"
															? "text-cyan-200/90"
															: driveNativeTruthStatus.tone === "warning"
																? "text-amber-200/90"
																: "text-rose-200/90"
													}`}
												>
													{driveNativeTruthStatus.message}
												</div>
											) : null}
											{(() => {
												const chips = buildAxisHealthChips(driveAxis);
												if (chips.length === 0) {
													return null;
												}
												return (
													<div
														className="mt-1 flex max-w-[14rem] flex-wrap gap-1"
														data-testid={`axis-health-chips-j${jointNumber}`}
													>
														{chips.map((chip) => (
															<span
																key={chip.key}
																className={`rounded px-1.5 py-0.5 text-[10px] leading-none ${AXIS_HEALTH_CHIP_TONE_CLASSES[chip.tone]}`}
															>
																{chip.label}
															</span>
														))}
													</div>
												);
											})()}
										</div>
										<div className="ml-auto grid grid-cols-3 gap-1">
											<button
												type="button"
												className="rounded border border-slate-700/70 bg-slate-950/60 px-2 py-1 text-xs font-semibold text-slate-200 transition hover:border-slate-500/80 hover:text-white disabled:cursor-not-allowed disabled:opacity-50"
												onClick={() => {
													void handleJointStep(jointIndex, -1);
												}}
												disabled={jogDisabled}
												title={
													!drivePowerReady && requiresExplicitDrivePower
														? `Power up the drives before jogging J${jointNumber}`
														: hasDisplayOrStaleFeedback
															? `Jog J${jointNumber} by -${jointStepLabel} degrees`
															: `Live joint feedback is required before jogging J${jointNumber}`
												}
											>
												-{jointStepLabel}°
											</button>
											<button
												type="button"
												className="rounded border border-slate-700/70 bg-slate-950/60 px-2 py-1 text-xs font-semibold text-slate-200 transition hover:border-slate-500/80 hover:text-white disabled:cursor-not-allowed disabled:opacity-50"
												onClick={() => {
													void handleJointStep(jointIndex, +1);
												}}
												disabled={jogDisabled}
												title={
													!drivePowerReady && requiresExplicitDrivePower
														? `Power up the drives before jogging J${jointNumber}`
														: hasDisplayOrStaleFeedback
															? `Jog J${jointNumber} by +${jointStepLabel} degrees`
															: `Live joint feedback is required before jogging J${jointNumber}`
												}
											>
												+{jointStepLabel}°
											</button>
											{showSoftwareZeroButton ? (
												<button
													type="button"
													className="rounded border border-amber-500/40 bg-amber-400/10 px-2 py-1 text-xs font-semibold text-amber-100 transition hover:border-amber-300/60 hover:bg-amber-300/15 disabled:cursor-not-allowed disabled:opacity-50"
													onClick={() => {
														void handleJointZero(jointIndex);
													}}
													disabled={zeroDisabled}
													title={
														hasLiveFeedback
															? `Capture current J${jointNumber} position as logical zero`
															: `Live joint feedback is required before zero capture on J${jointNumber}`
													}
												>
													{isPending && pendingJointAction === `zero-${jointIndex}` ? "..." : "Zero"}
												</button>
											) : null}
											<button
												type="button"
												className="rounded border border-cyan-500/40 bg-cyan-400/10 px-2 py-1 text-xs font-semibold text-cyan-100 transition hover:border-cyan-300/60 hover:bg-cyan-300/15 disabled:cursor-not-allowed disabled:opacity-50"
												onClick={() => {
													void handleJointNativeHome(jointIndex);
												}}
												disabled={nativeHomeDisabled}
												title={
													nativeHomeBusy
														? (
															nativeHomeActiveForJoint
																? `Drive-native home is still running for J${jointNumber}. Wait for the persistence step to finish.`
																: `Another drive-native home is still running. Wait for it to finish before homing J${jointNumber}.`
														)
														: hasLiveFeedback
															? `Run commissioning drive-native home for J${jointNumber} at the current pose`
															: hasDriveAxisFeedback
																? `Run commissioning drive-native home for J${jointNumber} using live drive telemetry. Canonical joint angles are currently unavailable.`
																: `Drive telemetry is required before commissioning drive-native home on J${jointNumber}`
												}
											>
												{isPending && pendingJointAction === `native-home-${jointIndex}` ? "..." : "Drive Home"}
											</button>
										</div>
									</div>
								);
							})}
						</div>
					</>
				) : null}
			</div>
			<div className="flex items-center justify-between">
				<button
					className="rounded bg-rose-600 px-3 py-2 text-white shadow hover:brightness-110 disabled:opacity-60"
					onClick={() => post("/control/stop")}
				>
					STOP
				</button>
				<button
					className="rounded border border-slate-600 bg-slate-800 px-3 py-2 hover:bg-slate-700 disabled:opacity-60"
					onClick={async () => {
						// Pause jog to avoid fighting the absolute move
						if (jogEnabled) {
							await stopJog();
						}
						await runDiscreteMotionCommand("home", async () => {
							const result = await post("/control/home") as MotionStatusResponse | null;
							if (result) {
								setMotionStatus(result);
							}
						});
					}}
					disabled={motionControlsBusy}
				>
					Home
				</button>
				<button
					className="rounded border border-slate-600 bg-slate-800 px-3 py-2 hover:bg-slate-700 disabled:opacity-60"
					onClick={async () => {
						if (jogEnabled) {
							await stopJog();
						}
						await runDiscreteMotionCommand("rest", async () => {
							const result = await post("/control/rest") as MotionStatusResponse | null;
							if (result) {
								setMotionStatus(result);
							}
						});
					}}
					disabled={motionControlsBusy}
				>
					Rest
				</button>
			</div>
		</>
	);
	return splitStatusSections ? (
		<div className="pointer-events-auto w-full space-y-3 text-slate-100">
			{showStatusSections ? drivePowerSection : null}
			{showStatusSections ? motionStateSection : null}
			<div className="w-full">
				{controlsAreCollapsible ? (
					<button
						type="button"
						onClick={onToggleControlsCollapsed}
						className="flex w-full items-center justify-between border border-slate-700/60 bg-slate-900/80 px-3 py-2 text-left transition hover:border-slate-500/70"
					>
						<span className="text-xs font-semibold uppercase tracking-[0.22em] text-cyan-200/80">
							Controls
						</span>
						<span className="text-[10px] font-semibold uppercase tracking-[0.16em] text-slate-300">
							{controlsCollapsed ? "Show" : "Hide"}
						</span>
					</button>
				) : (
					<div className="border border-slate-700/60 bg-slate-900/80 px-3 py-2 text-xs font-semibold uppercase tracking-[0.22em] text-cyan-200/80">
						Controls
					</div>
				)}
				{controlsAreCollapsible ? (
					controlsCollapsed ? null : (
						<div className="mt-2 border border-slate-700/60 bg-slate-900/80 p-3">
							{controlsBody}
						</div>
					)
				) : (
					<div className="mt-2 border border-slate-700/60 bg-slate-900/80 p-3">
						{controlsBody}
					</div>
				)}
			</div>
		</div>
	) : (
		<div className="pointer-events-auto w-full border border-slate-700/60 bg-slate-900/80 p-4 text-slate-100">
			<div className="mb-2 text-xs font-semibold uppercase tracking-[0.25em] text-cyan-200/80">
				Robot Control
			</div>
			{showStatusSections ? drivePowerSection : null}
			{showStatusSections ? motionStateSection : null}
			{controlsBody}
		</div>
	);
}

export default ControlPanel;


