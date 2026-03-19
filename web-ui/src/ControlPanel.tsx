import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { DEFAULT_SPEED_SLIDER } from "./uiConstants";

type Props = {
	apiHost: string;
	onError?: (message: string) => void;
};

type JointInfoResponse = {
	arm_deg?: number[];
	gripper_deg?: number;
};

type CommissioningStatus = {
	tone: "info" | "success" | "error";
	message: string;
};

const JOINT_STEP_OPTIONS_DEG = [0.25, 1, 5] as const;

function expSliderToMultiplier(v: number): number {
	// v in [0..1000] → 10^((t*2)-1) with t=v/1000
	const t = Math.max(0, Math.min(1000, v)) / 1000;
	const expVal = (t * 2) - 1;
	let mult = Math.pow(10, expVal);
	if (mult < 0.1) mult = 0.1;
	if (mult > 10) mult = 10;
	return mult;
}

export function ControlPanel({ apiHost, onError }: Props) {
	const [speedVal, setSpeedVal] = useState<number>(DEFAULT_SPEED_SLIDER); // 0..1000
	const speedMult = useMemo(() => expSliderToMultiplier(speedVal), [speedVal]);
	const [grip, setGrip] = useState<number>(0);
	const gripTimerRef = useRef<number | null>(null);
	const [busy, setBusy] = useState<boolean>(false);
	const [jointAnglesDeg, setJointAnglesDeg] = useState<number[]>([]);
	const [jointFeedbackError, setJointFeedbackError] = useState<string | null>(null);
	const [jointStepDeg, setJointStepDeg] = useState<number>(1);
	const [pendingJointAction, setPendingJointAction] = useState<string | null>(null);
	const [commissioningStatus, setCommissioningStatus] = useState<CommissioningStatus | null>(null);
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
	const keepaliveMs = 200;
	const linCountsRef = useRef<{ x: number; y: number; z: number }>({ x: 0, y: 0, z: 0 });
	const angCountsRef = useRef<{ x: number; y: number; z: number }>({ x: 0, y: 0, z: 0 });

	const reportRequestError = useCallback((err: unknown, fallback: string, silent: boolean) => {
		const msg = (err as Error)?.message || fallback;
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
				const msg = await res.text();
				throw new Error(msg || `${res.status} ${res.statusText}`);
			}
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
				const msg = await res.text();
				throw new Error(msg || `${res.status} ${res.statusText}`);
			}
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
		setJointFeedbackError(null);
		return true;
	}, [getJson]);

	useEffect(() => {
		let disposed = false;
		const tick = async () => {
			const payload = await getJson<JointInfoResponse>("/info/joints", true);
			if (disposed) {
				return;
			}
			if (!payload || !Array.isArray(payload.arm_deg)) {
				setJointFeedbackError("Waiting for joint feedback...");
				return;
			}
			setJointAnglesDeg(payload.arm_deg.map((value) => Number(value)));
			setJointFeedbackError(null);
		};
		void tick();
		const timer = window.setInterval(() => {
			void tick();
		}, 500);
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
		const changed =
			v[0] !== last[0] || v[1] !== last[1] || v[2] !== last[2] ||
			v[3] !== last[3] || v[4] !== last[4] || v[5] !== last[5];
		if (!changed && now - lastSentAtRef.current < keepaliveMs) {
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
			}, 20);
		}
	}, [jogEnabled, post, deadman]);

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

	const handleJointStep = useCallback(async (jointIndex: number, direction: 1 | -1) => {
		if (pendingJointAction) {
			return;
		}
		const jointNumber = jointIndex + 1;
		const signedStepDeg = Number((jointStepDeg * direction).toFixed(3));
		setPendingJointAction(`jog-${jointIndex}`);
		setCommissioningStatus({
			tone: "info",
			message: `Jogging J${jointNumber} by ${signedStepDeg > 0 ? "+" : ""}${signedStepDeg.toFixed(3)} deg...`,
		});
		try {
			if (jogEnabled) {
				await stopJog();
			}
			const result = await post("/control/joint-jog", {
				joint: jointNumber,
				delta_deg: signedStepDeg,
				wait_for_idle: true,
			});
			if (result) {
				const refreshed = await refreshJointAngles(false);
				setCommissioningStatus(
					refreshed
						? {
							tone: "success",
							message: `J${jointNumber} step complete. Live joint feedback refreshed.`,
						}
						: {
							tone: "error",
							message: `J${jointNumber} moved, but live feedback refresh did not succeed.`,
						},
				);
			} else {
				setCommissioningStatus({
					tone: "error",
					message: `Failed to jog J${jointNumber}. Check API/controller connectivity.`,
				});
			}
		} finally {
			setPendingJointAction(null);
		}
	}, [pendingJointAction, jogEnabled, stopJog, post, jointStepDeg, refreshJointAngles]);

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
					message: "Live joint feedback refreshed. You can now jog and capture zero from the current pose.",
				}
				: {
					tone: "error",
					message: "Joint feedback unavailable. Do not capture zero until live angles are visible.",
				},
		);
	}, [refreshJointAngles]);

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
				closed: true,
			});
		},
		[post, speedMult, linBaseMmS],
	);

	const performIncrementalAngularJog = useCallback(
		async (axis: "roll" | "pitch" | "yaw", direction: 1 | -1) => {
			// Use the user-configured angular base value from the UI as the incremental step size.
			const angleDeg = angBaseDegS * direction;
			await post("/control/rotate", {
				axis,
				angle_deg: angleDeg,
			});
		},
		[post, angBaseDegS],
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
					<span className="rounded border border-amber-500/30 bg-amber-400/10 px-2 py-0.5 text-[10px] font-semibold uppercase tracking-[0.18em] text-amber-200/90">
						RTCore cap 100 RPM
					</span>
				</div>
				<div className="mb-2 text-[11px] leading-relaxed text-slate-400">
					Use small relative steps while aligning joints for zero capture. This panel assumes the RTCore setup-time
					safety clamp remains at <code className="rounded bg-slate-900/80 px-1 py-0.5 text-slate-200">--max-rpm 100</code>.
				</div>
				<div className="mb-2 rounded border border-amber-500/20 bg-amber-400/5 px-2 py-2 text-[11px] leading-relaxed text-amber-100/90">
					Zero capture writes a persistent logical offset for the selected joint. Refresh feedback first, jog the joint
					into alignment, then capture zero only when the live angle shown for that joint matches what you expect.
				</div>
				<div className="mb-2 flex items-center justify-between gap-2">
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
						const controlsDisabled = pendingJointAction !== null;
						const commissioningDisabled = controlsDisabled || !hasLiveFeedback;
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
										disabled={commissioningDisabled}
										title={
											hasLiveFeedback
												? `Jog J${jointNumber} by -${jointStepDeg} degrees`
												: `Live joint feedback is required before jogging J${jointNumber}`
										}
									>
										-{jointStepDeg}°
									</button>
									<button
										type="button"
										className="rounded border border-slate-700/70 bg-slate-950/60 px-2 py-1 text-xs font-semibold text-slate-200 transition hover:border-slate-500/80 hover:text-white disabled:cursor-not-allowed disabled:opacity-50"
										onClick={() => {
											void handleJointStep(jointIndex, +1);
										}}
										disabled={commissioningDisabled}
										title={
											hasLiveFeedback
												? `Jog J${jointNumber} by +${jointStepDeg} degrees`
												: `Live joint feedback is required before jogging J${jointNumber}`
										}
									>
										+{jointStepDeg}°
									</button>
									<button
										type="button"
										className="rounded border border-amber-500/40 bg-amber-400/10 px-2 py-1 text-xs font-semibold text-amber-100 transition hover:border-amber-300/60 hover:bg-amber-300/15 disabled:cursor-not-allowed disabled:opacity-50"
										onClick={() => {
											void handleJointZero(jointIndex);
										}}
										disabled={commissioningDisabled}
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
			</div>
			<div className="flex items-center justify-between">
				<button
					className="rounded bg-rose-600 px-3 py-2 text-white shadow hover:brightness-110 disabled:opacity-60"
					onClick={() => post("/control/stop")}
					disabled={busy}
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
						await post("/control/home");
					}}
					disabled={busy}
				>
					Home
				</button>
				<button
					className="rounded border border-slate-600 bg-slate-800 px-3 py-2 hover:bg-slate-700 disabled:opacity-60"
					onClick={async () => {
						if (jogEnabled) {
							await stopJog();
						}
						await post("/control/rest");
					}}
					disabled={busy}
				>
					Rest
				</button>
			</div>
		</div>
	);
}

export default ControlPanel;


