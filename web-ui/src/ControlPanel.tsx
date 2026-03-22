import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { DEFAULT_SPEED_SLIDER } from "./uiConstants";

type Props = {
	apiHost: string;
	driveFaults?: DriveFaultSnapshot | null;
	activeServoBackend?: string | null;
	onJointFeedback?: (anglesDeg: number[], gripperDeg?: number) => void;
	onError?: (message: string) => void;
};

type JointInfoResponse = {
	arm_deg?: number[];
	arm_rad?: number[];
	gripper_deg?: number;
	gripper_rad?: number;
};

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
};

type ApiCommandPerformance = {
	count?: number;
	ok_count?: number;
	error_count?: number;
	timeout_count?: number;
	avg_round_trip_ms?: number;
	max_round_trip_ms?: number;
	last_round_trip_ms?: number;
};

type ApiUdpPerformance = {
	last_command?: string | null;
	last_result?: string | null;
	last_round_trip_ms?: number | null;
	by_command?: Record<string, ApiCommandPerformance>;
};

type RollingTimingMetric = {
	count?: number;
	avg_ms?: number;
	max_ms?: number;
	last_ms?: number;
	slow_over_5ms?: number;
	slow_over_20ms?: number;
	overrun_count?: number;
	max_overrun_ms?: number;
	last_overrun_ms?: number;
};

type ControllerCommandPerformance = {
	count?: number;
	last_dispatch_ms?: number;
	avg_dispatch_ms?: number;
	max_dispatch_ms?: number;
	last_interarrival_ms?: number | null;
	avg_interarrival_ms?: number;
	max_interarrival_ms?: number;
	interarrival_count?: number;
};

type ControllerUdpPerformance = {
	last_command?: string | null;
	last_peer?: string | null;
	last_dispatch_ms?: number | null;
	last_receive_age_s?: number | null;
	dispatch_ms?: RollingTimingMetric;
	interarrival_ms?: RollingTimingMetric;
	command_counts?: Record<string, number>;
	per_command?: Record<string, ControllerCommandPerformance>;
};

type JogPerformance = {
	control_frequency_hz?: number;
	execution_policy?: string;
	rtcore_owned?: boolean;
	last_velocity_command_age_s?: number | null;
	velocity_updates?: {
		count?: number;
		gap_count?: number;
		avg_gap_ms?: number;
		max_gap_ms?: number;
		last_gap_ms?: number | null;
		zero_velocity_updates?: number;
		nonzero_velocity_updates?: number;
	};
	loop?: RollingTimingMetric;
	stages?: {
		feedback_read_ms?: RollingTimingMetric;
		ik_solve_ms?: RollingTimingMetric;
		command_send_ms?: RollingTimingMetric;
	};
};

type RtcorePerformance = {
	rt_last_jitter_ns?: number | null;
	rt_max_abs_jitter_ns?: number | null;
	rt_overrun_count?: number | null;
	motion_last_update_age_ms?: number | null;
};

type PerformanceResponse = {
	collected_at?: string;
	api_udp?: ApiUdpPerformance;
	controller?: {
		udp?: ControllerUdpPerformance;
		jog?: JogPerformance;
		motion_state?: string;
		is_jogging?: boolean;
		last_command_age_s?: number;
		command_link_stale?: boolean;
	};
	rtcore?: RtcorePerformance | null;
};

const LIVE_JOINT_FEEDBACK_POLL_MS = 20;
const PERFORMANCE_POLL_MS = 500;

type CommissioningStatus = {
	tone: "info" | "success" | "error";
	message: string;
};

type DriveFaultDetail = {
	error_code_hex?: string;
	profile_id?: string;
	decoded?: boolean;
	code?: string;
	name?: string;
	class?: string | null;
	resettable?: boolean;
};

type DriveFaultAxis = {
	axis: number;
	logical_joint?: number | null;
	ds402_state: string;
	statusword: number;
	error_code: number;
	fault?: DriveFaultDetail | null;
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
	axis_enable_mask_hex?: string;
	op_enabled_axes?: number;
	num_axes?: number;
	faulted_axes?: number;
	axes?: DriveFaultAxis[];
};

const JOINT_STEP_OPTIONS_DEG = [0.25, 1, 5] as const;
const MIN_CUSTOM_JOINT_STEP_DEG = 0.001;

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
	const code = axis.fault?.code?.trim();
	const name = axis.fault?.name?.trim();
	if (code) {
		parts.push(code);
	}
	if (name) {
		parts.push(name);
	}
	if (axis.fault?.resettable === true) {
		parts.push("resettable");
	}
	return parts.join(" | ");
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

function formatMs(value: number | null | undefined, digits = 2): string {
	if (typeof value !== "number" || !Number.isFinite(value)) {
		return "--";
	}
	return `${value.toFixed(digits)} ms`;
}

function formatAgeSeconds(value: number | null | undefined, digits = 2): string {
	if (typeof value !== "number" || !Number.isFinite(value)) {
		return "--";
	}
	return `${value.toFixed(digits)} s`;
}

function formatNsAsUs(value: number | null | undefined, digits = 2): string {
	if (typeof value !== "number" || !Number.isFinite(value)) {
		return "--";
	}
	return `${(value / 1000).toFixed(digits)} us`;
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
					return String((payload.detail as { message: string }).message);
				}
				if (typeof payload.detail === "string") {
					return payload.detail;
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

export function ControlPanel({ apiHost, driveFaults, activeServoBackend, onJointFeedback, onError }: Props) {
	const [speedVal, setSpeedVal] = useState<number>(DEFAULT_SPEED_SLIDER); // 0..1000
	const speedMult = useMemo(() => expSliderToMultiplier(speedVal), [speedVal]);
	const [grip, setGrip] = useState<number>(0);
	const gripTimerRef = useRef<number | null>(null);
	const [jointAnglesDeg, setJointAnglesDeg] = useState<number[]>([]);
	const [jointFeedbackError, setJointFeedbackError] = useState<string | null>(null);
	const [jointStepDeg, setJointStepDeg] = useState<number>(1);
	const [customJointStepInput, setCustomJointStepInput] = useState<string>("");
	const [pendingJointAction, setPendingJointAction] = useState<string | null>(null);
	const [commissioningStatus, setCommissioningStatus] = useState<CommissioningStatus | null>(null);
	const [commissioningExpanded, setCommissioningExpanded] = useState<boolean>(false);
	const [motionStatus, setMotionStatus] = useState<MotionStatusResponse | null>(null);
	const [performanceSnapshot, setPerformanceSnapshot] = useState<PerformanceResponse | null>(null);
	const activeDriveFaultAxes = useMemo(
		() =>
			(driveFaults?.axes ?? []).filter(
				(axis) => axis.error_code !== 0 || axis.ds402_state === "Fault" || axis.ds402_state === "FaultReactionActive",
			),
		[driveFaults],
	);
	const driveControlBackend = (driveFaults?.servo_backend ?? activeServoBackend ?? "").trim().toLowerCase();
	const requiresExplicitDrivePower = driveControlBackend === "ethercat_rtcore";
	const isDrivePowerActive = (driveFaults?.driver_state ?? "").trim().toUpperCase() === "ACTIVE";
	const drivePowerReady = !requiresExplicitDrivePower || isDrivePowerActive;
	const motionStateName = (motionStatus?.state ?? motionStatus?.execution?.state_name ?? "idle").trim().toLowerCase();
	const motionTrajectoryId = typeof motionStatus?.trajectory_id === "number" && motionStatus.trajectory_id > 0
		? motionStatus.trajectory_id
		: (typeof motionStatus?.execution?.active_traj_id === "number" && motionStatus.execution.active_traj_id > 0
			? motionStatus.execution.active_traj_id
			: null);
	const motionBusy = motionStateName === "accepted" || motionStateName === "queued" || motionStateName === "executing";
	// Realtime jog state
	const [jogEnabled, setJogEnabled] = useState<boolean>(false);
	const [deadman, setDeadman] = useState<boolean>(true);
	const [linBaseMmS, setLinBaseMmS] = useState<number>(50);
	const [angBaseDegS, setAngBaseDegS] = useState<number>(15);
	const [lastJogCommand, setLastJogCommand] = useState<[number, number, number, number, number, number]>([0, 0, 0, 0, 0, 0]);
	const jogTimerRef = useRef<number | null>(null);
	const sendJogTickRef = useRef<() => Promise<void>>(async () => {});
	const lastSentRef = useRef<[number, number, number, number, number, number]>([0, 0, 0, 0, 0, 0]);
	const lastSentAtRef = useRef<number>(0);
	const lastRequestErrorRef = useRef<string | null>(null);
	const keepaliveMs = 200;
	const jogTickIntervalMs = 50;
	const linCountsRef = useRef<{ x: number; y: number; z: number }>({ x: 0, y: 0, z: 0 });
	const angCountsRef = useRef<{ x: number; y: number; z: number }>({ x: 0, y: 0, z: 0 });
	const jointStepLabel = formatStepDegrees(jointStepDeg);
	const perfApiUdp = performanceSnapshot?.api_udp;
	const perfController = performanceSnapshot?.controller;
	const perfControllerUdp = perfController?.udp;
	const perfJog = perfController?.jog;
	const perfRtcore = performanceSnapshot?.rtcore;
	const perfMoveApi = perfApiUdp?.by_command?.MOVE_LINE_RELATIVE;
	const perfWaitApi = perfApiUdp?.by_command?.WAIT_FOR_IDLE;
	const perfJogApi = perfApiUdp?.by_command?.SET_JOG_VELOCITY;
	const perfMoveController = perfControllerUdp?.per_command?.MOVE_LINE_RELATIVE;
	const perfWaitController = perfControllerUdp?.per_command?.WAIT_FOR_IDLE;
	const perfJogController = perfControllerUdp?.per_command?.SET_JOG_VELOCITY;
	const perfSnapshotAgeSeconds = useMemo(() => {
		if (!performanceSnapshot?.collected_at) {
			return null;
		}
		const collectedAtMs = Date.parse(performanceSnapshot.collected_at);
		if (!Number.isFinite(collectedAtMs)) {
			return null;
		}
		return Math.max(0, (Date.now() - collectedAtMs) / 1000);
	}, [performanceSnapshot]);

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
		const payload = await getJson<JointInfoResponse>("/info/joints", silent);
		if (!payload || !Array.isArray(payload.arm_deg)) {
			if (!silent) {
				setJointFeedbackError("Joint feedback unavailable.");
			}
			return false;
		}
		const nextAngles = payload.arm_deg.map((value) => Number(value));
		setJointAnglesDeg(nextAngles);
		try {
			onJointFeedback?.(
				nextAngles,
				typeof payload.gripper_deg === "number" ? Number(payload.gripper_deg) : undefined,
			);
		} catch {
			// Ignore visualizer sync errors so commissioning feedback stays usable.
		}
		setJointFeedbackError(null);
		return true;
	}, [getJson, onJointFeedback]);

	useEffect(() => {
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
		}, LIVE_JOINT_FEEDBACK_POLL_MS);
		return () => {
			disposed = true;
			window.clearInterval(timer);
		};
	}, [refreshJointAngles]);

	useEffect(() => {
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
	}, [getJson]);

	useEffect(() => {
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
			const payload = await getJson<PerformanceResponse>("/debug/performance", true);
			inFlight = false;
			if (disposed || !payload) {
				return;
			}
			setPerformanceSnapshot(payload);
		};
		void tick();
		const timer = window.setInterval(() => {
			void tick();
		}, PERFORMANCE_POLL_MS);
		return () => {
			disposed = true;
			window.clearInterval(timer);
		};
	}, [getJson]);

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

	const sendJogTick = useCallback(async () => {
		const now = Date.now();
		const v = computeJogVector();
		const last = lastSentRef.current;
		const isZeroVector =
			v[0] === 0 && v[1] === 0 && v[2] === 0 &&
			v[3] === 0 && v[4] === 0 && v[5] === 0;
		const changed =
			v[0] !== last[0] || v[1] !== last[1] || v[2] !== last[2] ||
			v[3] !== last[3] || v[4] !== last[4] || v[5] !== last[5];
		if (!changed && (isZeroVector || now - lastSentAtRef.current < keepaliveMs)) {
			return;
		}
		const commandPayload: [number, number, number, number, number, number] = [
			Number(v[0].toFixed(6)),
			Number(v[1].toFixed(6)),
			Number(v[2].toFixed(6)),
			Number(v[3].toFixed(3)),
			Number(v[4].toFixed(3)),
			Number(v[5].toFixed(3)),
		];
		await post("/control/jog/velocity", {
			vx: commandPayload[0],
			vy: commandPayload[1],
			vz: commandPayload[2],
			v_roll: commandPayload[3],
			v_pitch: commandPayload[4],
			v_yaw: commandPayload[5],
		});
		setLastJogCommand(commandPayload);
		lastSentRef.current = v;
		lastSentAtRef.current = now;
	}, [computeJogVector, post]);

	// Keep the interval callback hot-swapped with latest slider/deadman/base values.
	useEffect(() => {
		sendJogTickRef.current = sendJogTick;
	}, [sendJogTick]);

	const ensureJogStarted = useCallback(async () => {
		if (!jogEnabled) {
			setJogEnabled(true);
			await post("/control/jog/start");
			await post("/control/jog/deadman", { enabled: deadman });
			// start timer
			if (jogTimerRef.current) {
				window.clearInterval(jogTimerRef.current);
			}
			jogTimerRef.current = window.setInterval(() => {
				sendJogTickRef.current().catch(() => {});
			}, jogTickIntervalMs);
		}
	}, [jogEnabled, post, deadman, jogTickIntervalMs]);

	const stopJog = useCallback(async () => {
		setJogEnabled(false);
		if (jogTimerRef.current) {
			window.clearInterval(jogTimerRef.current);
			jogTimerRef.current = null;
		}
		linCountsRef.current = { x: 0, y: 0, z: 0 };
		angCountsRef.current = { x: 0, y: 0, z: 0 };
		lastSentRef.current = [0, 0, 0, 0, 0, 0];
		setLastJogCommand([0, 0, 0, 0, 0, 0]);
		await post("/control/jog/velocity", { vx: 0, vy: 0, vz: 0, v_roll: 0, v_pitch: 0, v_yaw: 0 });
		await post("/control/jog/stop");
	}, [post]);

	const onDeadmanToggle = useCallback(async (enabled: boolean) => {
		setDeadman(enabled);
		await post("/control/jog/deadman", { enabled });
	}, [post]);
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
			const result = await post("/control/reset-faults", isSingleJoint ? { joint: jointNumber } : undefined);
			if (result) {
				const refreshed = await refreshJointAngles(false);
				setCommissioningStatus(
					refreshed
						? {
							tone: "success",
							message: isSingleJoint
								? `Drive fault reset requested for J${jointNumber}. Live joint feedback refreshed.`
								: "Drive fault reset requested for all axes. Live joint feedback refreshed.",
						}
						: {
							tone: "success",
							message: isSingleJoint
								? `Drive fault reset requested for J${jointNumber}. Refresh joint feedback after the drive recovers.`
								: "Drive fault reset requested for all axes. Refresh joint feedback after the drives recover.",
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
	}, [pendingJointAction, post, refreshJointAngles]);

	const handlePowerUpDrives = useCallback(async () => {
		if (pendingJointAction) {
			return;
		}
		const confirmed = window.confirm(
			"Power up RTCore-controlled drives now? This will arm and enable the configured axes.",
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
			const result = await post("/control/power-up");
			if (result) {
				const refreshed = await refreshJointAngles(false);
				setCommissioningStatus(
					refreshed
						? {
							tone: "success",
							message: "Drive power-up requested. Live joint feedback refreshed.",
						}
						: {
							tone: "success",
							message: "Drive power-up requested. Refresh joint feedback after the axes finish enabling.",
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
	}, [pendingJointAction, post, refreshJointAngles]);

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
			const result = await post("/control/power-down");
			if (result) {
				const refreshed = await refreshJointAngles(false);
				setCommissioningStatus(
					refreshed
						? {
							tone: "success",
							message: "Drive power-down requested. Live joint feedback refreshed.",
						}
						: {
							tone: "success",
							message: "Drive power-down requested. Refresh joint feedback after the axes disarm.",
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
	}, [pendingJointAction, post, refreshJointAngles]);

	// ------------------------
	// Incremental jog helpers
	// ------------------------
	const performIncrementalLinearJog = useCallback(
		async (axis: "x" | "y" | "z", direction: 1 | -1) => {
			// Use the user-configured linear base value from the UI as the incremental step size.
			// The field is labeled in mm/s for realtime jog, but here we treat the numeric value
			// as a distance in millimeters for a single incremental move.
			const stepMeters = (linBaseMmS / 1000.0) * direction;
			const dx = axis === "x" ? stepMeters : 0;
			const dy = axis === "y" ? stepMeters : 0;
			const dz = axis === "z" ? stepMeters : 0;
			await post("/control/move-line-relative", {
				dx,
				dy,
				dz,
				speed_multiplier: speedMult,
				closed: false,
			});
		},
		[post, speedMult, linBaseMmS],
	);

	const performIncrementalAngularJog = useCallback(
		async (axis: "roll" | "pitch" | "yaw", direction: 1 | -1) => {
			// Use the user-configured angular base value from the UI as the incremental step size.
			const angleDeg = angBaseDegS * direction;
			const commandedAngularDegS = Math.max(1e-3, effectiveAngularDegS);
			await post("/control/rotate", {
				axis,
				angle_deg: angleDeg,
				duration_s: Math.abs(angleDeg) / commandedAngularDegS,
			});
		},
		[post, angBaseDegS, effectiveAngularDegS],
	);

	const changeLinearCount = useCallback(async (axis: "x" | "y" | "z", delta: number) => {
		// Realtime jog: adjust the active axis counts and send an immediate tick.
		// Jog must already be started via the Start button; we do not auto-start here,
		// so that a simple button tap can remain purely incremental when realtime is off.
		await ensureJogStarted();
		linCountsRef.current = { ...linCountsRef.current, [axis]: linCountsRef.current[axis] + delta };
		// immediate tick to avoid delay
		await sendJogTick();
	}, [ensureJogStarted, sendJogTick]);

	const changeAngularCount = useCallback(async (axis: "x" | "y" | "z", delta: number) => {
		// Realtime jog: adjust the active axis counts and send an immediate tick.
		await ensureJogStarted();
		angCountsRef.current = { ...angCountsRef.current, [axis]: angCountsRef.current[axis] + delta };
		await sendJogTick();
	}, [ensureJogStarted, sendJogTick]);

	const onPress = useCallback((fn: () => void) => (e: React.PointerEvent<HTMLButtonElement>) => {
		(e.currentTarget as HTMLButtonElement).setPointerCapture(e.pointerId);
		fn();
	}, []);
	const onRelease = useCallback((fn: () => void) => (e: React.PointerEvent<HTMLButtonElement>) => {
		try {
			(e.currentTarget as HTMLButtonElement).releasePointerCapture(e.pointerId);
		} catch {}
		fn();
	}, []);

	return (
		<div className="pointer-events-auto w-full rounded-xl border border-slate-700/60 bg-slate-900/80 p-4 text-slate-100 shadow-lg shadow-slate-900/40 backdrop-blur">
			<div className="mb-2 text-xs font-semibold uppercase tracking-[0.25em] text-cyan-200/80">
				Robot Control
			</div>
			<div className="mb-3 rounded-lg border border-cyan-500/25 bg-cyan-500/5 p-3">
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
						? `Backend ${driveFaults.servo_backend ?? "unknown"} | driver ${driveFaults.driver_state ?? "unknown"} | EtherCAT ${driveFaults.ethercat_master_state ?? "unknown"} | RTCore ${driveFaults.rtcore_state ?? "unknown"} | enable mask ${driveFaults.axis_enable_mask_hex ?? "unknown"} | op-enabled ${driveFaults.op_enabled_axes ?? 0}/${driveFaults.num_axes ?? 0}`
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
						disabled={pendingJointAction !== null || !requiresExplicitDrivePower || isDrivePowerActive}
						title="Explicitly arm and enable RTCore-controlled axes"
					>
						{pendingJointAction === "power-up" ? "..." : "Power Up Drives"}
					</button>
					<button
						type="button"
						className="rounded border border-rose-500/35 bg-rose-400/10 px-2 py-2 text-[11px] font-semibold text-rose-100 transition hover:border-rose-300/60 hover:bg-rose-300/15 disabled:cursor-not-allowed disabled:opacity-50"
						onClick={() => {
							void handlePowerDownDrives();
						}}
						disabled={pendingJointAction !== null || !requiresExplicitDrivePower || !isDrivePowerActive}
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
			<div className="mb-3 rounded-lg border border-slate-700/60 bg-slate-950/30 p-3">
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
			</div>
			<div className="mb-3 rounded-lg border border-violet-500/20 bg-violet-500/5 p-3">
				<div className="mb-2 flex items-center justify-between gap-2 text-xs text-violet-100/90">
					<span className="font-semibold uppercase tracking-[0.18em]">Timing Diagnostics</span>
					<span className="rounded border border-violet-400/30 bg-violet-400/10 px-2 py-0.5 text-[10px] font-semibold uppercase tracking-[0.16em] text-violet-100">
						{performanceSnapshot ? `live ${formatAgeSeconds(perfSnapshotAgeSeconds)}` : "waiting"}
					</span>
				</div>
				<div className="mb-2 text-[11px] leading-relaxed text-slate-300">
					Recorded automatically whenever the UI calls the API. This panel reads the live `/debug/performance`
					snapshot, so no extra toggle is needed to capture timing while you test from the UI.
				</div>
				<div className="mb-3 rounded border border-slate-700/60 bg-slate-950/40 px-2 py-2 text-[10px] text-slate-300">
					<div className="font-semibold text-slate-200">UI -&gt; API command timing</div>
					<div className="mt-1 tabular-nums">
						Jog velocity RTT avg/max: {formatMs(perfJogApi?.avg_round_trip_ms)} / {formatMs(perfJogApi?.max_round_trip_ms)}
					</div>
					<div className="tabular-nums">
						Move request RTT avg/max: {formatMs(perfMoveApi?.avg_round_trip_ms)} / {formatMs(perfMoveApi?.max_round_trip_ms)}
					</div>
					<div className="tabular-nums">
						Wait-for-idle RTT avg/max: {formatMs(perfWaitApi?.avg_round_trip_ms)} / {formatMs(perfWaitApi?.max_round_trip_ms)}
					</div>
					<div className="mt-1 text-slate-400">
						`MOVE_LINE_RELATIVE` includes planning before the ACK returns. `WAIT_FOR_IDLE` includes the full motion time.
					</div>
				</div>
				<div className="mb-3 rounded border border-slate-700/60 bg-slate-950/40 px-2 py-2 text-[10px] text-slate-300">
					<div className="font-semibold text-slate-200">Controller UDP dispatch</div>
					<div className="mt-1 tabular-nums">
						Last command: {perfControllerUdp?.last_command ?? "--"} | last dispatch {formatMs(perfControllerUdp?.last_dispatch_ms)}
					</div>
					<div className="tabular-nums">
						Global dispatch avg/max: {formatMs(perfControllerUdp?.dispatch_ms?.avg_ms)} / {formatMs(perfControllerUdp?.dispatch_ms?.max_ms)}
					</div>
					<div className="tabular-nums">
						Global interarrival last/avg: {formatMs(perfControllerUdp?.interarrival_ms?.last_ms)} / {formatMs(perfControllerUdp?.interarrival_ms?.avg_ms)}
					</div>
					<div className="tabular-nums">
						Move dispatch avg/max: {formatMs(perfMoveController?.avg_dispatch_ms)} / {formatMs(perfMoveController?.max_dispatch_ms)}
					</div>
					<div className="tabular-nums">
						Jog dispatch avg/max: {formatMs(perfJogController?.avg_dispatch_ms)} / {formatMs(perfJogController?.max_dispatch_ms)}
					</div>
					<div className="tabular-nums">
						Slow dispatch counts &gt;5 ms / &gt;20 ms: {perfControllerUdp?.dispatch_ms?.slow_over_5ms ?? 0} / {perfControllerUdp?.dispatch_ms?.slow_over_20ms ?? 0}
					</div>
					<div className="tabular-nums">
						Controller command-link stale: {perfController?.command_link_stale ? "yes" : "no"} | last receive age {formatAgeSeconds(perfControllerUdp?.last_receive_age_s)}
					</div>
				</div>
				<div className="mb-3 rounded border border-slate-700/60 bg-slate-950/40 px-2 py-2 text-[10px] text-slate-300">
					<div className="font-semibold text-slate-200">Jog loop internals</div>
					<div className="mt-1 tabular-nums">
						Loop avg/max: {formatMs(perfJog?.loop?.avg_ms)} / {formatMs(perfJog?.loop?.max_ms)}
					</div>
					<div className="tabular-nums">
						Loop overshoot max: {formatMs(perfJog?.loop?.max_overrun_ms)} | overrun count {perfJog?.loop?.overrun_count ?? 0}
					</div>
					<div className="tabular-nums">
						Velocity update gap avg/max: {formatMs(perfJog?.velocity_updates?.avg_gap_ms)} / {formatMs(perfJog?.velocity_updates?.max_gap_ms)}
					</div>
					<div className="tabular-nums">
						Feedback read avg: {formatMs(perfJog?.stages?.feedback_read_ms?.avg_ms)} | IK avg: {formatMs(perfJog?.stages?.ik_solve_ms?.avg_ms)} | send avg: {formatMs(perfJog?.stages?.command_send_ms?.avg_ms)}
					</div>
					<div className="tabular-nums">
						Jog command age: {formatAgeSeconds(perfJog?.last_velocity_command_age_s)} | controller thinks jogging: {perfController?.is_jogging ? "yes" : "no"}
					</div>
				</div>
				<div className="rounded border border-slate-700/60 bg-slate-950/40 px-2 py-2 text-[10px] text-slate-300">
					<div className="font-semibold text-slate-200">RTCore health</div>
					<div className="mt-1 tabular-nums">
						RT jitter current/max: {formatNsAsUs(perfRtcore?.rt_last_jitter_ns)} / {formatNsAsUs(perfRtcore?.rt_max_abs_jitter_ns)}
					</div>
					<div className="tabular-nums">
						RT overruns: {perfRtcore?.rt_overrun_count ?? 0} | motion update age {formatMs(perfRtcore?.motion_last_update_age_ms)}
					</div>
				</div>
			</div>
			{/* Removed step move blocks; unified under realtime jog below */}
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
							onClick={async () => (jogEnabled ? await stopJog() : await ensureJogStarted())}
						>
							{jogEnabled ? "Stop" : "Start"}
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
				</div>
				<div className="grid grid-cols-3 gap-1">
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
						onPointerCancel={() => {
							if (jogEnabled) {
								changeLinearCount("x", -1).catch(() => {});
							}
						}}
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
						onPointerCancel={() => {
							if (jogEnabled) {
								changeLinearCount("y", -1).catch(() => {});
							}
						}}
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
						onPointerCancel={() => {
							if (jogEnabled) {
								changeLinearCount("z", -1).catch(() => {});
							}
						}}
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
						onPointerCancel={() => {
							if (jogEnabled) {
								changeLinearCount("x", +1).catch(() => {});
							}
						}}
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
						onPointerCancel={() => {
							if (jogEnabled) {
								changeLinearCount("y", +1).catch(() => {});
							}
						}}
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
						onPointerCancel={() => {
							if (jogEnabled) {
								changeLinearCount("z", +1).catch(() => {});
							}
						}}
					>
						-Z
					</button>
				</div>
				<div className="mt-2 grid grid-cols-3 gap-1">
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
						onPointerCancel={() => {
							if (jogEnabled) {
								changeAngularCount("x", -1).catch(() => {});
							}
						}}
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
						onPointerCancel={() => {
							if (jogEnabled) {
								changeAngularCount("y", -1).catch(() => {});
							}
						}}
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
						onPointerCancel={() => {
							if (jogEnabled) {
								changeAngularCount("z", -1).catch(() => {});
							}
						}}
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
						onPointerCancel={() => {
							if (jogEnabled) {
								changeAngularCount("x", +1).catch(() => {});
							}
						}}
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
						onPointerCancel={() => {
							if (jogEnabled) {
								changeAngularCount("y", +1).catch(() => {});
							}
						}}
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
						onPointerCancel={() => {
							if (jogEnabled) {
								changeAngularCount("z", +1).catch(() => {});
							}
						}}
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
								Drives are currently disarmed. Use the <span className="font-semibold">Drive Power</span> section above before jogging.
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
						{jointFeedbackError ? (
							<div className="mb-2 rounded border border-slate-700/60 bg-slate-900/70 px-2 py-1.5 text-[11px] text-slate-400">
								{jointFeedbackError}
							</div>
						) : null}
						{commissioningStatus ? (
							<div
								className={`mb-2 rounded border px-2 py-1.5 text-[11px] ${
									commissioningStatus.tone === "success"
										? "border-emerald-500/30 bg-emerald-400/10 text-emerald-100"
										: commissioningStatus.tone === "error"
											? "border-rose-500/30 bg-rose-400/10 text-rose-100"
											: "border-cyan-500/20 bg-cyan-400/10 text-cyan-100"
								}`}
							>
								{commissioningStatus.message}
							</div>
						) : null}
						<div className="space-y-1">
							{Array.from({ length: jointAnglesDeg.length > 0 ? jointAnglesDeg.length : 6 }, (_, jointIndex) => {
								const jointNumber = jointIndex + 1;
								const angleDeg = jointAnglesDeg[jointIndex];
								const angleLabel = Number.isFinite(angleDeg) ? `${angleDeg.toFixed(2)}°` : "--";
								const isPending = pendingJointAction === `jog-${jointIndex}` || pendingJointAction === `zero-${jointIndex}`;
								const hasLiveFeedback = Number.isFinite(angleDeg);
								const controlsDisabled = pendingJointAction !== null || motionBusy;
								const jogDisabled = controlsDisabled || !hasLiveFeedback || !drivePowerReady;
								const zeroDisabled = controlsDisabled || !hasLiveFeedback;
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
														: hasLiveFeedback
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
														: hasLiveFeedback
															? `Jog J${jointNumber} by +${jointStepLabel} degrees`
															: `Live joint feedback is required before jogging J${jointNumber}`
												}
											>
												+{jointStepLabel}°
											</button>
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
						const result = await post("/control/home") as MotionStatusResponse | null;
						if (result) {
							setMotionStatus(result);
						}
					}}
					disabled={motionBusy}
				>
					Home
				</button>
				<button
					className="rounded border border-slate-600 bg-slate-800 px-3 py-2 hover:bg-slate-700 disabled:opacity-60"
					onClick={async () => {
						if (jogEnabled) {
							await stopJog();
						}
						const result = await post("/control/rest") as MotionStatusResponse | null;
						if (result) {
							setMotionStatus(result);
						}
					}}
					disabled={motionBusy}
				>
					Rest
				</button>
			</div>
		</div>
	);
}

export default ControlPanel;


