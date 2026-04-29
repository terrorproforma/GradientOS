import { cleanup, fireEvent, render, screen, waitFor } from "@testing-library/react";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { ControlPanel, ControlPanelRuntimeHeader, preferredTelemetryJointAnglesRad } from "./ControlPanel";
import { LiveStateProvider } from "./liveState";

type FetchRecord = {
	method: string;
	pathname: string;
	body: unknown;
};

function mockJsonResponse(body: unknown, status = 200): Response {
	return {
		ok: status >= 200 && status < 300,
		status,
		statusText: status === 200 ? "OK" : "ERROR",
		headers: new Headers({ "content-type": "application/json" }),
		json: async () => body,
		text: async () => JSON.stringify(body),
	} as Response;
}

function sleep(ms: number): Promise<void> {
	return new Promise((resolve) => {
		window.setTimeout(resolve, ms);
	});
}

describe("ControlPanel jog session lifecycle", () => {
	let fetchCalls: FetchRecord[];

	beforeEach(() => {
		fetchCalls = [];
		Object.defineProperty(HTMLElement.prototype, "setPointerCapture", {
			configurable: true,
			value: vi.fn(),
		});
		Object.defineProperty(HTMLElement.prototype, "releasePointerCapture", {
			configurable: true,
			value: vi.fn(),
		});
		vi.stubGlobal("fetch", vi.fn(async (input: string | URL | Request, init?: RequestInit) => {
			const url = typeof input === "string"
				? input
				: input instanceof URL
					? input.toString()
					: input.url;
			const method = init?.method ?? "GET";
			const parsed = new URL(url, "http://localhost");
			const body = typeof init?.body === "string" && init.body.length > 0
				? JSON.parse(init.body)
				: undefined;
			fetchCalls.push({
				method,
				pathname: parsed.pathname,
				body,
			});

			if (parsed.pathname === "/info/joints" || parsed.pathname === "/info/joints-detailed") {
				return mockJsonResponse({
					arm_deg: [0, 0, 0, 0, 0, 0],
					arm_display_deg: [0, 0, 0, 0, 0, 0],
					gripper_deg: 0,
					read_source: "live_feedback",
				});
			}
			if (parsed.pathname === "/control/motion-status") {
				return mockJsonResponse({
					status: "ok",
					state: "idle",
					safe_for_power_transition: true,
					power_transition_blockers: [],
					power_transition_blocker_details: [],
					execution: {
						state_name: "idle",
						active_mode_name: "idle",
						controller_thread_running: false,
						rtcore_status_present: true,
						queue_depth: 0,
						queue_capacity: 4096,
						motion_done: true,
						stale_command: false,
						safe_for_power_transition: true,
						power_transition_blockers: [],
						power_transition_blocker_details: [],
					},
				});
			}
			if (parsed.pathname === "/control/jog/session/start") {
				return mockJsonResponse({
					status: "ok",
					session: {
						session_id: "session-1",
						state: "active",
						last_seq_received: 0,
					},
				});
			}
			if (parsed.pathname === "/control/jog/session/update") {
				return mockJsonResponse({
					status: "ok",
					session: {
						session_id: "session-1",
						state: "active",
						last_seq_received: 1,
					},
				});
			}
			if (parsed.pathname === "/control/jog/session/stop") {
				return mockJsonResponse({
					status: "ok",
					session: {
						session_id: "session-1",
						state: "stopped",
						last_stop_reason: "ui-release",
					},
				});
			}
			return mockJsonResponse({});
		}));
	});

	afterEach(() => {
		cleanup();
		vi.restoreAllMocks();
		vi.unstubAllGlobals();
	});

	it("does not open a jog session while armed but idle", async () => {
		render(<ControlPanel apiHost="" />);

		fireEvent.click(screen.getByRole("button", { name: "Arm Jog" }));
		await sleep(400);

		const jogPosts = fetchCalls.filter((call) =>
			call.method === "POST" && call.pathname.startsWith("/control/jog/session/"),
		);
		expect(jogPosts).toEqual([]);
	});

	it("starts on pointer down and stops on final release", async () => {
		render(<ControlPanel apiHost="" />);

		fireEvent.click(screen.getByRole("button", { name: "Arm Jog" }));
		await waitFor(() => {
			expect(screen.getByRole("button", { name: "Disarm Jog" })).toBeTruthy();
		});
		const plusX = screen.getByRole("button", { name: "+X" });

		fireEvent.pointerDown(plusX, { pointerId: 1 });
		await waitFor(() => {
			expect(fetchCalls.some((call) => call.method === "POST" && call.pathname === "/control/jog/session/start")).toBe(true);
		});

		fireEvent.pointerUp(plusX, { pointerId: 1 });
		await waitFor(() => {
			expect(fetchCalls.some((call) => call.method === "POST" && call.pathname === "/control/jog/session/stop")).toBe(true);
		});

		const jogPosts = fetchCalls.filter((call) => call.method === "POST" && call.pathname.startsWith("/control/jog/session/"));
		expect(jogPosts[0]?.pathname).toBe("/control/jog/session/start");
		expect((jogPosts[0]?.body as { lease_timeout_s?: number } | undefined)?.lease_timeout_s).toBe(1);
		expect(jogPosts[jogPosts.length - 1]?.pathname).toBe("/control/jog/session/stop");
	});

	it("keeps an active held jog alive even if the jog interval is stale", async () => {
		const realSetInterval = window.setInterval.bind(window);
		vi.spyOn(window, "setInterval").mockImplementation(((handler: TimerHandler, timeout?: number, ...args: unknown[]) => {
			if (timeout === 50) {
				return 424242;
			}
			return realSetInterval(handler, timeout, ...args);
		}) as typeof window.setInterval);

		render(<ControlPanel apiHost="" />);

		fireEvent.click(screen.getByRole("button", { name: "Arm Jog" }));
		await waitFor(() => {
			expect(screen.getByRole("button", { name: "Disarm Jog" })).toBeTruthy();
		});
		fireEvent.pointerDown(screen.getByRole("button", { name: "+X" }), { pointerId: 1 });

		await waitFor(() => {
			expect(fetchCalls.some((call) => call.method === "POST" && call.pathname === "/control/jog/session/start")).toBe(true);
		});
		await waitFor(() => {
			const updatePosts = fetchCalls.filter((call) =>
				call.method === "POST" && call.pathname === "/control/jog/session/update",
			);
			expect(updatePosts.length).toBeGreaterThanOrEqual(2);
		});

		const jogPosts = fetchCalls.filter((call) =>
			call.method === "POST" && call.pathname.startsWith("/control/jog/session/"),
		);
		expect(jogPosts[0]?.pathname).toBe("/control/jog/session/start");
		expect(jogPosts.some((call) => (
			call.pathname === "/control/jog/session/update"
			&& (call.body as { session_id?: string } | undefined)?.session_id === "session-1"
		))).toBe(true);
	});

	it("recovers an expired active hold without posting a stop or power-down", async () => {
		let startCount = 0;
		let updateCount = 0;
		vi.stubGlobal("fetch", vi.fn(async (input: string | URL | Request, init?: RequestInit) => {
			const url = typeof input === "string"
				? input
				: input instanceof URL
					? input.toString()
					: input.url;
			const method = init?.method ?? "GET";
			const parsed = new URL(url, "http://localhost");
			const body = typeof init?.body === "string" && init.body.length > 0
				? JSON.parse(init.body)
				: undefined;
			fetchCalls.push({ method, pathname: parsed.pathname, body });

			if (parsed.pathname === "/info/joints" || parsed.pathname === "/info/joints-detailed") {
				return mockJsonResponse({
					arm_deg: [0, 0, 0, 0, 0, 0],
					arm_display_deg: [0, 0, 0, 0, 0, 0],
					gripper_deg: 0,
					read_source: "live_feedback",
				});
			}
			if (parsed.pathname === "/control/motion-status") {
				return mockJsonResponse({
					status: "ok",
					state: "idle",
					safe_for_power_transition: true,
					power_transition_blockers: [],
					power_transition_blocker_details: [],
					execution: {
						state_name: "idle",
						active_mode_name: "idle",
						controller_thread_running: false,
						rtcore_status_present: true,
						queue_depth: 0,
						queue_capacity: 4096,
						motion_done: true,
						stale_command: false,
						safe_for_power_transition: true,
						power_transition_blockers: [],
						power_transition_blocker_details: [],
					},
				});
			}
			if (parsed.pathname === "/control/jog/session/start") {
				startCount += 1;
				return mockJsonResponse({
					status: "ok",
					session: {
						session_id: `session-${startCount}`,
						state: "active",
						last_seq_received: 0,
					},
				});
			}
			if (parsed.pathname === "/control/jog/session/update") {
				updateCount += 1;
				if (updateCount === 1) {
					return mockJsonResponse({
						detail: {
							code: "SESSION_NOT_FOUND",
							message: "Jog session not found.",
						},
					}, 404);
				}
				return mockJsonResponse({
					status: "ok",
					session: {
						session_id: `session-${startCount}`,
						state: "active",
						last_seq_received: updateCount,
					},
				});
			}
			if (parsed.pathname === "/control/jog/session/stop") {
				return mockJsonResponse({ status: "ok", session: { state: "stopped" } });
			}
			return mockJsonResponse({});
		}));

		render(<ControlPanel apiHost="" />);
		fireEvent.click(screen.getByRole("button", { name: "Arm Jog" }));
		const plusX = screen.getByRole("button", { name: "+X" });
		fireEvent.pointerDown(plusX, { pointerId: 1 });

		await waitFor(() => {
			expect(startCount).toBeGreaterThanOrEqual(2);
		});

		const postedPaths = fetchCalls
			.filter((call) => call.method === "POST")
			.map((call) => call.pathname);
		expect(postedPaths).not.toContain("/control/jog/session/stop");
		expect(postedPaths).not.toContain("/control/power-down");
	});

	it("uses operator display joint angles for frontend feedback", async () => {
		const onJointFeedback = vi.fn();
		vi.stubGlobal("fetch", vi.fn(async (input: string | URL | Request, init?: RequestInit) => {
			const url = typeof input === "string"
				? input
				: input instanceof URL
					? input.toString()
					: input.url;
			const parsed = new URL(url, "http://localhost");
			if (parsed.pathname === "/info/joints" || parsed.pathname === "/info/joints-detailed") {
				return mockJsonResponse({
					arm_deg: [10, 20, 30, 40, 50, 60],
					arm_display_deg: [1, 2, 3, 4, 5, 6],
					gripper_deg: 7,
					read_source: "unavailable",
				});
			}
			if (parsed.pathname === "/control/motion-status") {
				return mockJsonResponse({
					status: "ok",
					state: "idle",
					safe_for_power_transition: true,
					power_transition_blockers: [],
					power_transition_blocker_details: [],
					execution: {
						state_name: "idle",
						active_mode_name: "idle",
						controller_thread_running: false,
						rtcore_status_present: true,
						queue_depth: 0,
						queue_capacity: 4096,
						motion_done: true,
						stale_command: false,
						safe_for_power_transition: true,
						power_transition_blockers: [],
						power_transition_blocker_details: [],
					},
				});
			}
			return mockJsonResponse({});
		}));

		render(<ControlPanel apiHost="" onJointFeedback={onJointFeedback} />);

		await waitFor(() => {
			expect(onJointFeedback).toHaveBeenCalledWith([1, 2, 3, 4, 5, 6], 7);
		});
	});

	it("does not poll detailed joints while monitor telemetry is fresh", async () => {
		const onJointFeedback = vi.fn();
		render(
			<LiveStateProvider
				value={{
					apiHost: "",
					latest: {
						timestamp: Date.now(),
						raw: "monitor",
						joints: [0, 0, 0, 0, 0, 0],
						display_joints: [0, 0, 0, 0, 0, 0],
					},
					motionStatus: null,
					setMotionStatus: vi.fn(),
					isConnected: true,
					monitorLastMessageAtMs: Date.now(),
					monitorFreshnessMs: 0,
					isMonitorFresh: true,
					monitorError: null,
					apiHealthError: null,
				}}
			>
				<ControlPanel apiHost="" onJointFeedback={onJointFeedback} />
			</LiveStateProvider>,
		);

		await sleep(250);

		expect(fetchCalls.some((call) => call.pathname === "/info/joints-detailed")).toBe(false);
		expect(onJointFeedback).not.toHaveBeenCalled();
	});

	it("does not fall back to canonical joint angles when display angles are missing", async () => {
		const onJointFeedback = vi.fn();
		vi.stubGlobal("fetch", vi.fn(async (input: string | URL | Request, init?: RequestInit) => {
			const url = typeof input === "string"
				? input
				: input instanceof URL
					? input.toString()
					: input.url;
			const parsed = new URL(url, "http://localhost");
			if (parsed.pathname === "/info/joints" || parsed.pathname === "/info/joints-detailed") {
				return mockJsonResponse({
					arm_deg: [10, 20, 30, 40, 50, 60],
					gripper_deg: 7,
					read_source: "live_feedback",
				});
			}
			if (parsed.pathname === "/control/motion-status") {
				return mockJsonResponse({
					status: "ok",
					state: "idle",
					safe_for_power_transition: true,
					power_transition_blockers: [],
					power_transition_blocker_details: [],
					execution: {
						state_name: "idle",
						active_mode_name: "idle",
						controller_thread_running: false,
						rtcore_status_present: true,
						queue_depth: 0,
						queue_capacity: 4096,
						motion_done: true,
						stale_command: false,
						safe_for_power_transition: true,
						power_transition_blockers: [],
						power_transition_blocker_details: [],
					},
				});
			}
			return mockJsonResponse({});
		}));

		render(<ControlPanel apiHost="" onJointFeedback={onJointFeedback} />);

		await waitFor(() => {
			expect(onJointFeedback).toHaveBeenCalledWith([], undefined);
		});
		expect(onJointFeedback).not.toHaveBeenCalledWith([10, 20, 30, 40, 50, 60], 7);
	});

	it("shows partial operator display truth without falling back when raw canonical truth is unavailable", async () => {
		const onJointFeedback = vi.fn();
		vi.stubGlobal("fetch", vi.fn(async (input: string | URL | Request, init?: RequestInit) => {
			const url = typeof input === "string"
				? input
				: input instanceof URL
					? input.toString()
					: input.url;
			const parsed = new URL(url, "http://localhost");
			if (parsed.pathname === "/info/joints" || parsed.pathname === "/info/joints-detailed") {
				return mockJsonResponse({
					arm_deg: [10, 20, 30, 40, 50, 60],
					arm_display_deg: [1, null, 3, null, 5, 6],
					gripper_deg: 7,
					read_source: "unavailable",
				});
			}
			if (parsed.pathname === "/control/motion-status") {
				return mockJsonResponse({
					status: "ok",
					state: "idle",
					safe_for_power_transition: true,
					power_transition_blockers: [],
					power_transition_blocker_details: [],
					execution: {
						state_name: "idle",
						active_mode_name: "idle",
						controller_thread_running: false,
						rtcore_status_present: true,
						queue_depth: 0,
						queue_capacity: 4096,
						motion_done: true,
						stale_command: false,
						safe_for_power_transition: true,
						power_transition_blockers: [],
						power_transition_blocker_details: [],
					},
				});
			}
			return mockJsonResponse({});
		}));

		render(<ControlPanel apiHost="" onJointFeedback={onJointFeedback} />);
		fireEvent.click(screen.getByRole("button", { name: "Show" }));

		await waitFor(() => {
			const driveHomeButtons = screen.getAllByRole("button", { name: "Drive Home" });
			expect(driveHomeButtons).toHaveLength(6);
			expect(driveHomeButtons[0].hasAttribute("disabled")).toBe(false);
			expect(driveHomeButtons[1].hasAttribute("disabled")).toBe(true);
			expect(driveHomeButtons[2].hasAttribute("disabled")).toBe(false);
			expect(driveHomeButtons[3].hasAttribute("disabled")).toBe(true);
			expect(driveHomeButtons[4].hasAttribute("disabled")).toBe(false);
			expect(driveHomeButtons[5].hasAttribute("disabled")).toBe(false);
		});
		expect(screen.getAllByText("1.00°").length).toBeGreaterThan(0);
		expect(screen.getAllByText("3.00°").length).toBeGreaterThan(0);
		expect(screen.getAllByText("5.00°").length).toBeGreaterThan(0);
		expect(screen.getAllByText("6.00°").length).toBeGreaterThan(0);
		expect(screen.getAllByText("--").length).toBeGreaterThan(0);
		expect(onJointFeedback).toHaveBeenCalledWith([], undefined);
		expect(onJointFeedback).not.toHaveBeenCalledWith([10, 20, 30, 40, 50, 60], 7);
	});

	it("keeps a fixed commissioning message rail while native-home messages change", async () => {
		render(
			<ControlPanel
				apiHost=""
				driveFaults={{
					servo_backend: "ethercat_rtcore",
					driver_state: "DISARMED",
					axis_enable_mask: 0x0,
					native_home_active_axis_mask: 0x2,
					axes: [
						{
							axis: 1,
							logical_joint: 2,
							ds402_state: "SwitchOnDisabled",
							statusword: 0x1650,
							error_code: 0,
							native_home_state: 0,
							native_home_state_name: "idle",
							native_home_active: true,
							native_home_last_abort_code: 0,
							native_home_last_abort_code_hex: "0x00000000",
						},
					],
				}}
			/>,
		);

		fireEvent.click(screen.getByRole("button", { name: "Show" }));

		await waitFor(() => {
			expect(screen.getByText(/Drive-native home still running for J2/i)).toBeTruthy();
			const slots = screen.getAllByTestId("commissioning-message-slot");
			expect(slots).toHaveLength(3);
			expect(slots[0].className).toContain("h-9");
			expect(slots[0].getAttribute("aria-hidden")).toBeNull();
			expect(slots[1].getAttribute("aria-hidden")).toBe("true");
			expect(slots[2].getAttribute("aria-hidden")).toBe("true");
		});
	});

	it("fills per-joint display gaps from live canonical angles so a 360-deg-offset joint stays visible", async () => {
		// Regression for the 2026-04-17 J6 incident: after a full-shaft
		// excursion the display-mode command-roundtrip gate can reject a
		// single joint (J6) even though raw/canonical truth stays live and
		// shaft-frame-consistent. Before the fix the commissioning panel
		// flickered between "5 angles + J6 '--'" and "all '--' + Waiting
		// for joint feedback..." ~10 times a second because the fallback
		// polling effect was keyed off the whole monitor telemetry object
		// instead of a stable availability boolean. The panel must now show
		// J6's canonical angle in place of the missing display slot so the
		// operator can read the state and reach the Drive Home button.
		const onJointFeedback = vi.fn();
		vi.stubGlobal("fetch", vi.fn(async (input: string | URL | Request) => {
			const url = typeof input === "string"
				? input
				: input instanceof URL
					? input.toString()
					: input.url;
			const parsed = new URL(url, "http://localhost");
			if (parsed.pathname === "/info/joints" || parsed.pathname === "/info/joints-detailed") {
				return mockJsonResponse({
					arm_deg: [5.45, -4.91, 0.06, -0.01, -0.02, 360.01],
					arm_display_deg: [5.45, -4.91, 0.06, -0.01, -0.02, null],
					read_source: "live_feedback",
					raw_canonical_joint_truth_available: true,
					display_joint_truth_available: false,
					canonical_joint_truth_available: true,
				});
			}
			if (parsed.pathname === "/control/motion-status") {
				return mockJsonResponse({
					status: "ok",
					state: "idle",
					safe_for_power_transition: true,
					power_transition_blockers: [],
					power_transition_blocker_details: [],
					execution: {
						state_name: "idle",
						active_mode_name: "idle",
						controller_thread_running: false,
						rtcore_status_present: true,
						queue_depth: 0,
						queue_capacity: 4096,
						motion_done: true,
						stale_command: false,
						safe_for_power_transition: true,
						power_transition_blockers: [],
						power_transition_blocker_details: [],
					},
				});
			}
			return mockJsonResponse({});
		}));

		render(<ControlPanel apiHost="" onJointFeedback={onJointFeedback} />);
		fireEvent.click(screen.getByRole("button", { name: "Show" }));

		await waitFor(() => {
			expect(screen.getAllByText("5.45°").length).toBeGreaterThan(0);
			expect(screen.getAllByText("360.01°").length).toBeGreaterThan(0);
		});
		expect(screen.queryAllByText("--")).toHaveLength(0);
	});

	it("does not fall back to canonical when display truth is partial but canonical is not authoritative", async () => {
		// Counter-regression for the above: if raw canonical truth is NOT
		// live (e.g. read_source="unavailable" after a cold boot with no
		// persisted anchor), missing display slots must stay as "--"
		// rather than being papered over by cached `arm_deg` values. This
		// preserves the existing "don't leak cached canonical into the
		// operator display" contract.
		vi.stubGlobal("fetch", vi.fn(async (input: string | URL | Request) => {
			const url = typeof input === "string"
				? input
				: input instanceof URL
					? input.toString()
					: input.url;
			const parsed = new URL(url, "http://localhost");
			if (parsed.pathname === "/info/joints" || parsed.pathname === "/info/joints-detailed") {
				return mockJsonResponse({
					arm_deg: [5.45, -4.91, 0.06, -0.01, -0.02, 360.01],
					arm_display_deg: [5.45, -4.91, 0.06, -0.01, -0.02, null],
					read_source: "unavailable",
					raw_canonical_joint_truth_available: false,
				});
			}
			if (parsed.pathname === "/control/motion-status") {
				return mockJsonResponse({ status: "ok", state: "idle" });
			}
			return mockJsonResponse({});
		}));

		render(<ControlPanel apiHost="" />);
		fireEvent.click(screen.getByRole("button", { name: "Show" }));

		await waitFor(() => {
			expect(screen.getAllByText("5.45°").length).toBeGreaterThan(0);
			expect(screen.queryAllByText("360.01°")).toHaveLength(0);
			expect(screen.getAllByText("--").length).toBeGreaterThan(0);
		});
	});

	it("keeps drive-home enabled when canonical angles are unavailable but drive telemetry is live", async () => {
		vi.stubGlobal("fetch", vi.fn(async (input: string | URL | Request, init?: RequestInit) => {
			const url = typeof input === "string"
				? input
				: input instanceof URL
					? input.toString()
					: input.url;
			const parsed = new URL(url, "http://localhost");
			if (parsed.pathname === "/info/joints" || parsed.pathname === "/info/joints-detailed") {
				return mockJsonResponse({
					arm_deg: [10, 20, 30, 40, 50, 60],
					arm_display_deg: [null, null, null, null, null, null],
					gripper_deg: 7,
					read_source: "unavailable",
				});
			}
			if (parsed.pathname === "/control/motion-status") {
				return mockJsonResponse({
					status: "ok",
					state: "idle",
					safe_for_power_transition: false,
					power_transition_blockers: ["coordinate_system_invalid"],
					power_transition_blocker_details: [
						{
							code: "coordinate_system_invalid",
							message: "Drive-native coordinate system is invalid; run Drive Home before power-up.",
							truth_unavailable_joints: [1, 2, 3, 4, 5, 6],
							statuswords: ["0x1650"],
						},
					],
					execution: {
						state_name: "idle",
						active_mode_name: "idle",
						controller_thread_running: false,
						rtcore_status_present: true,
						queue_depth: 0,
						queue_capacity: 4096,
						motion_done: true,
						stale_command: false,
						safe_for_power_transition: false,
						power_transition_blockers: ["coordinate_system_invalid"],
						power_transition_blocker_details: [
							{
								code: "coordinate_system_invalid",
								message: "Drive-native coordinate system is invalid; run Drive Home before power-up.",
								truth_unavailable_joints: [1, 2, 3, 4, 5, 6],
								statuswords: ["0x1650"],
							},
						],
					},
				});
			}
			return mockJsonResponse({});
		}));

		render(
			<ControlPanel
				apiHost=""
				driveFaults={{
					servo_backend: "ethercat_rtcore",
					driver_state: "DISARMED",
					native_home_active_axis_mask: 0x0,
					axes: Array.from({ length: 6 }, (_, axis) => ({
						axis,
						logical_joint: axis + 1,
						ds402_state: "SwitchOnDisabled",
						statusword: 0x1650,
						error_code: 0,
						native_home_state: 0,
						native_home_state_name: "idle",
						native_home_last_abort_code: 0,
						native_home_last_abort_code_hex: "0x00000000",
					})),
				}}
			/>,
		);
		fireEvent.click(screen.getByRole("button", { name: "Show" }));

		await waitFor(() => {
			const driveHomeButtons = screen.getAllByRole("button", { name: "Drive Home" }) as HTMLButtonElement[];
			expect(driveHomeButtons).toHaveLength(6);
			for (const button of driveHomeButtons) {
				expect(button.disabled).toBe(false);
			}
			expect(driveHomeButtons[0].title).toContain("live drive telemetry");
		});
	});

	it("clears joint feedback when the detailed endpoint drops out", async () => {
		const onJointFeedback = vi.fn();
		let jointFetchCount = 0;
		vi.stubGlobal("fetch", vi.fn(async (input: string | URL | Request, init?: RequestInit) => {
			const url = typeof input === "string"
				? input
				: input instanceof URL
					? input.toString()
					: input.url;
			const parsed = new URL(url, "http://localhost");
			if (parsed.pathname === "/info/joints" || parsed.pathname === "/info/joints-detailed") {
				jointFetchCount += 1;
				if (jointFetchCount === 1) {
					return mockJsonResponse({
						arm_deg: [10, 20, 30, 40, 50, 60],
						arm_display_deg: [10, 20, 30, 40, 50, 60],
						gripper_deg: 7,
						read_source: "live_feedback",
					});
				}
				return mockJsonResponse({ detail: "temporary dropout" }, 503);
			}
			if (parsed.pathname === "/control/motion-status") {
				return mockJsonResponse({
					status: "ok",
					state: "idle",
					safe_for_power_transition: true,
					power_transition_blockers: [],
					power_transition_blocker_details: [],
					execution: {
						state_name: "idle",
						active_mode_name: "idle",
						controller_thread_running: false,
						rtcore_status_present: true,
						queue_depth: 0,
						queue_capacity: 4096,
						motion_done: true,
						stale_command: false,
						safe_for_power_transition: true,
						power_transition_blockers: [],
						power_transition_blocker_details: [],
					},
				});
			}
			return mockJsonResponse({});
		}));

		render(<ControlPanel apiHost="" onJointFeedback={onJointFeedback} />);

		await waitFor(() => {
			expect(onJointFeedback).toHaveBeenCalledWith([10, 20, 30, 40, 50, 60], 7);
		});
		await sleep(700);

		const clearedFeedback = onJointFeedback.mock.calls.some((call) =>
			Array.isArray(call[0]) && call[0].length === 0,
		);
		expect(jointFetchCount).toBeGreaterThan(1);
		expect(clearedFeedback).toBe(true);
	});

	it("sends controller-owned joint deltas without API-side joint baselining", async () => {
		render(<ControlPanel apiHost="" />);
		fireEvent.click(screen.getByRole("button", { name: "Show" }));

		await waitFor(() => {
			const firstJogButton = screen.getAllByRole("button", { name: "+1°" })[0] as HTMLButtonElement;
			expect(firstJogButton.disabled).toBe(false);
		});
		fireEvent.click(screen.getAllByRole("button", { name: "+1°" })[0]);

		await waitFor(() => {
			expect(fetchCalls.some((call) => call.pathname === "/control/joint-jog")).toBe(true);
		});
		const jogCall = fetchCalls.find((call) => call.pathname === "/control/joint-jog");
		expect(jogCall?.method).toBe("POST");
		expect(jogCall?.body).toMatchObject({
			joint: 1,
			delta_deg: 1,
			wait_for_idle: true,
		});
	});

	it("keeps stale telemetry joint samples renderable", () => {
		expect(preferredTelemetryJointAnglesRad({
			joints: [1, 0, 0, 0, 0, 0],
			joint_feedback_stale: true,
		})).toEqual([1, 0, 0, 0, 0, 0]);
	});

	it("blocks drive power-up when the runtime is unsafe", async () => {
		render(
			<ControlPanel
				apiHost=""
				driveFaults={{
					servo_backend: "ethercat_rtcore",
					driver_state: "DISARMED",
					axes: [],
				}}
				motionStatus={{
					status: "ok",
					state: "accepted",
					safe_for_power_transition: false,
					power_transition_blocker_details: [
						{
							code: "active_trajectory",
							message: "An RTCore trajectory is still latched or active.",
							active_traj_id: 9,
						},
					],
					execution: {
						state_name: "queued",
						active_mode_name: "trajectory_execute",
						active_traj_id: 9,
						queue_depth: 1,
						queue_capacity: 4096,
						motion_done: false,
						stale_command: false,
						safe_for_power_transition: false,
						power_transition_blockers: ["active_trajectory"],
						power_transition_blocker_details: [
							{
								code: "active_trajectory",
								message: "An RTCore trajectory is still latched or active.",
								active_traj_id: 9,
							},
						],
					},
				}}
			/>,
		);

		const button = screen.getByRole("button", { name: "Power Up Drives" }) as HTMLButtonElement;
		expect(button.disabled).toBe(true);
		expect(button.title).toContain("Active trajectory 9");
	});

	it("separates requested enable from actual drive feedback", async () => {
		render(
			<ControlPanel
				apiHost=""
				driveFaults={{
					servo_backend: "ethercat_rtcore",
					driver_state: "DISARMED",
					armed: 1,
					axis_enable_mask: 0x3f,
					axis_enable_mask_hex: "0x3f",
					enable_requested: true,
					requested_axes: 6,
					op_enabled_axes: 0,
					num_axes: 6,
					statusword_feedback_axes: 0,
					axes: [],
				}}
				motionStatus={{
					status: "ok",
					state: "idle",
					safe_for_power_transition: true,
					power_transition_blockers: [],
					power_transition_blocker_details: [],
					execution: {
						state_name: "idle",
						active_mode_name: "idle",
						queue_depth: 0,
						queue_capacity: 4096,
						motion_done: true,
						stale_command: false,
						safe_for_power_transition: true,
						power_transition_blockers: [],
						power_transition_blocker_details: [],
					},
				}}
			/>,
		);

		const powerUpButton = screen.getByRole("button", { name: "Power Up Drives" }) as HTMLButtonElement;
		const powerDownButton = screen.getByRole("button", { name: "Power Down Drives" }) as HTMLButtonElement;

		expect(powerUpButton.disabled).toBe(true);
		expect(powerUpButton.title).toContain("waiting for actual drive feedback");
		expect(powerDownButton.disabled).toBe(false);
		expect(screen.getByText("DISARMED")).toBeTruthy();
	});

	it("uses distinct drive-power and jog labels", async () => {
		render(
			<ControlPanel
				apiHost=""
				driveFaults={{
					servo_backend: "ethercat_rtcore",
					driver_state: "DISARMED",
					axes: [],
				}}
				motionStatus={{
					status: "ok",
					state: "idle",
					safe_for_power_transition: true,
					power_transition_blockers: [],
					power_transition_blocker_details: [],
					execution: {
						state_name: "idle",
						active_mode_name: "idle",
						queue_depth: 0,
						queue_capacity: 4096,
						motion_done: true,
						stale_command: false,
						safe_for_power_transition: true,
						power_transition_blockers: [],
						power_transition_blocker_details: [],
					},
				}}
			/>,
		);

		expect(screen.getByRole("button", { name: "Power Up Drives" })).toBeTruthy();
		expect(screen.getByRole("button", { name: "Arm Jog" })).toBeTruthy();
		expect(screen.queryByRole("button", { name: "Arm" })).toBeNull();
	});

	it("uses explicit power labels in the runtime header", async () => {
		render(
			<ControlPanelRuntimeHeader
				apiHost=""
				driveFaults={{
					servo_backend: "ethercat_rtcore",
					driver_state: "DISARMED",
					axes: [],
				}}
				motionStatus={{
					status: "ok",
					state: "idle",
					safe_for_power_transition: true,
					power_transition_blockers: [],
					power_transition_blocker_details: [],
					execution: {
						state_name: "idle",
						active_mode_name: "idle",
						queue_depth: 0,
						queue_capacity: 4096,
						motion_done: true,
						stale_command: false,
						safe_for_power_transition: true,
						power_transition_blockers: [],
						power_transition_blocker_details: [],
					},
				}}
			/>,
		);

		expect(screen.getByRole("button", { name: "Power Up" })).toBeTruthy();
		expect(screen.getByRole("button", { name: "Power Down" })).toBeTruthy();
		expect(screen.queryByRole("button", { name: "Arm" })).toBeNull();
	});

	it("shows CHECK instead of BLOCKED when only synchronization is unsettled while drives are disarmed", async () => {
		render(
			<ControlPanelRuntimeHeader
				apiHost=""
				driveFaults={{
					servo_backend: "ethercat_rtcore",
					driver_state: "DISARMED",
					axes: [],
				}}
				motionStatus={{
					status: "ok",
					state: "idle",
					safe_for_power_transition: false,
					power_transition_blockers: ["not_synchronized"],
					power_transition_blocker_details: [
						{
							code: "not_synchronized",
							message: "Live feedback is not synchronized yet; keep the drives disarmed.",
						},
					],
					execution: {
						state_name: "idle",
						active_mode_name: "idle",
						queue_depth: 0,
						queue_capacity: 4096,
						motion_done: true,
						stale_command: false,
						safe_for_power_transition: false,
						power_transition_blockers: ["not_synchronized"],
						power_transition_blocker_details: [
							{
								code: "not_synchronized",
								message: "Live feedback is not synchronized yet; keep the drives disarmed.",
							},
						],
					},
				}}
			/>,
		);

		expect(screen.getByText("CHECK")).toBeTruthy();
		expect(screen.queryByText("BLOCKED")).toBeNull();
	});

	it("shows BLOCKED with a native-home message when the drive coordinate system is invalid", async () => {
		render(
			<ControlPanelRuntimeHeader
				apiHost=""
				driveFaults={{
					servo_backend: "ethercat_rtcore",
					driver_state: "DISARMED",
					axes: [],
				}}
				motionStatus={{
					status: "ok",
					state: "idle",
					safe_for_power_transition: false,
					power_transition_blockers: ["coordinate_system_invalid"],
					power_transition_blocker_details: [
						{
							code: "coordinate_system_invalid",
							message: "Drive-native coordinate system is invalid; run Drive Home before power-up.",
							truth_unavailable_joints: [1, 2, 3, 4, 5, 6],
							statuswords: ["0x1650"],
						},
					],
					execution: {
						state_name: "idle",
						active_mode_name: "idle",
						queue_depth: 0,
						queue_capacity: 4096,
						motion_done: true,
						stale_command: false,
						safe_for_power_transition: false,
						power_transition_blockers: ["coordinate_system_invalid"],
						power_transition_blocker_details: [
							{
								code: "coordinate_system_invalid",
								message: "Drive-native coordinate system is invalid; run Drive Home before power-up.",
								truth_unavailable_joints: [1, 2, 3, 4, 5, 6],
								statuswords: ["0x1650"],
							},
						],
					},
				}}
			/>,
		);

		expect(screen.getByText("BLOCKED")).toBeTruthy();
		expect(screen.queryByText("CHECK")).toBeNull();
		const badge = screen.getByText("BLOCKED").closest("span");
		expect(badge?.getAttribute("title")).toMatch(/Drive coordinate system is invalid/i);
		expect(badge?.getAttribute("title")).toMatch(/Drive Home/i);
	});

	it("waits for a stable safe signal before showing SAFE in the runtime header while drives are disarmed", async () => {
		render(
			<ControlPanelRuntimeHeader
				apiHost=""
				driveFaults={{
					servo_backend: "ethercat_rtcore",
					driver_state: "DISARMED",
					axes: [],
				}}
				motionStatus={{
					status: "ok",
					state: "idle",
					safe_for_power_transition: true,
					power_transition_blockers: [],
					power_transition_blocker_details: [],
					execution: {
						state_name: "idle",
						active_mode_name: "idle",
						queue_depth: 0,
						queue_capacity: 4096,
						motion_done: true,
						stale_command: false,
						safe_for_power_transition: true,
						power_transition_blockers: [],
						power_transition_blocker_details: [],
					},
				}}
			/>,
		);

		expect(screen.getByText("CHECK")).toBeTruthy();
		expect(screen.queryByText("SAFE")).toBeNull();

		await sleep(650);

		await waitFor(() => {
			expect(screen.getByText("SAFE")).toBeTruthy();
		});
	});

	it("hides software zero by default and only shows it when enabled", async () => {
		const { rerender } = render(
			<ControlPanel
				apiHost=""
				driveFaults={{
					servo_backend: "ethercat_rtcore",
					driver_state: "DISARMED",
					axes: [],
				}}
			/>,
		);

		fireEvent.click(screen.getByRole("button", { name: "Show" }));
		expect(screen.queryByRole("button", { name: "Zero" })).toBeNull();
		expect(screen.getAllByRole("button", { name: "Drive Home" }).length).toBeGreaterThan(0);

		rerender(
			<ControlPanel
				apiHost=""
				showSoftwareZeroButton
				driveFaults={{
					servo_backend: "ethercat_rtcore",
					driver_state: "DISARMED",
					axes: [],
				}}
			/>,
		);

		expect(screen.getAllByRole("button", { name: "Zero" }).length).toBeGreaterThan(0);
	});

	it("shows per-joint native home status from drive telemetry", async () => {
		render(
			<ControlPanel
				apiHost=""
				driveFaults={{
					servo_backend: "ethercat_rtcore",
					driver_state: "DISARMED",
					axis_enable_mask: 0x0,
					axes: [
						{
							axis: 1,
							logical_joint: 2,
							ds402_state: "SwitchOnDisabled",
							statusword: 0x1638,
							error_code: 0,
							native_home_state: 2,
							native_home_state_name: "succeeded",
							native_home_position_offset: -244354,
							native_home_last_abort_code: 0,
							native_home_last_abort_code_hex: "0x00000000",
						},
					],
				}}
			/>,
		);

		fireEvent.click(screen.getByRole("button", { name: "Show" }));
		expect(screen.getByText(/Drive Home succeeded/i)).toBeTruthy();
		expect(screen.getByText(/axis currently disarmed/i)).toBeTruthy();
	});

	it("shows success when the live drive status is clean even if the reported abort is stale", async () => {
		render(
			<ControlPanel
				apiHost=""
				driveFaults={{
					servo_backend: "ethercat_rtcore",
					driver_state: "DISARMED",
					axis_enable_mask: 0x0,
					axes: [
						{
							axis: 1,
							logical_joint: 2,
							ds402_state: "SwitchOnDisabled",
							statusword: 0x9650,
							error_code: 0,
							native_home_state: 2,
							native_home_state_name: "succeeded",
							native_home_position_offset: 0,
							native_home_last_abort_code: 0,
							native_home_last_abort_code_hex: "0x00000000",
							native_home_state_reported: 3,
							native_home_state_reported_name: "failed",
							native_home_last_abort_code_reported: 0x06010002,
							native_home_last_abort_code_reported_hex: "0x06010002",
							native_home_verification_source: "statusword_bits12_15_clear13",
						},
					],
				}}
			/>,
		);

		fireEvent.click(screen.getByRole("button", { name: "Show" }));
		expect(screen.getByText(/Drive Home succeeded/i)).toBeTruthy();
		expect(screen.queryByText(/verification conflicted/i)).toBeNull();
	});

	it("renders per-axis health chips with A6-EC extended PDO telemetry", async () => {
		render(
			<ControlPanel
				apiHost=""
				driveFaults={{
					servo_backend: "ethercat_rtcore",
					driver_state: "DISARMED",
					axes: [
						{
							axis: 0,
							logical_joint: 1,
							ds402_state: "OperationEnabled",
							statusword: 0x1650,
							error_code: 0,
							bus_voltage_v: 47.8,
							igbt_temp_c: 42,
							motor_temp_c: 55,
							load_rate_pct: 23.5,
							position_error_counts: 12,
							motor_not_rotating_text: "ok",
							drive_not_ready_text: "ready",
						},
					],
				}}
			/>,
		);

		fireEvent.click(screen.getByRole("button", { name: "Show" }));

		const chipRow = screen.getByTestId("axis-health-chips-j1");
		expect(chipRow).toBeTruthy();
		// Voltage chip rounds to one decimal.
		expect(chipRow.textContent).toContain("47.8 V");
		expect(chipRow.textContent).toContain("IGBT 42°C");
		expect(chipRow.textContent).toContain("Motor 55°C");
		// Load is rounded to zero decimals.
		expect(chipRow.textContent).toContain("Load 24%");
		// Small position error stays below the chip threshold.
		expect(chipRow.textContent).not.toContain("PE ");
		// "ok"/"ready" decorations are hidden to avoid visual noise.
		expect(chipRow.textContent).not.toContain("ok");
		expect(chipRow.textContent).not.toContain("ready");
	});

	it("shows a warn-tone chip when IGBT temperature exceeds 70°C", async () => {
		render(
			<ControlPanel
				apiHost=""
				driveFaults={{
					servo_backend: "ethercat_rtcore",
					driver_state: "ACTIVE",
					axes: [
						{
							axis: 2,
							logical_joint: 3,
							ds402_state: "OperationEnabled",
							statusword: 0x1650,
							error_code: 0,
							igbt_temp_c: 75,
							bus_voltage_v: 48,
							load_rate_pct: 30,
						},
					],
				}}
			/>,
		);

		fireEvent.click(screen.getByRole("button", { name: "Show" }));

		const chipRow = screen.getByTestId("axis-health-chips-j3");
		const igbtChip = Array.from(chipRow.querySelectorAll("span")).find((span) =>
			span.textContent?.includes("IGBT 75°C"),
		);
		expect(igbtChip).toBeTruthy();
		expect(igbtChip?.className).toContain("amber");
	});

	it("shows error-tone chip when IGBT temperature exceeds 85°C", async () => {
		render(
			<ControlPanel
				apiHost=""
				driveFaults={{
					servo_backend: "ethercat_rtcore",
					driver_state: "ACTIVE",
					axes: [
						{
							axis: 3,
							logical_joint: 4,
							ds402_state: "OperationEnabled",
							statusword: 0x1650,
							error_code: 0,
							igbt_temp_c: 92,
							bus_voltage_v: 48,
							load_rate_pct: 40,
						},
					],
				}}
			/>,
		);

		fireEvent.click(screen.getByRole("button", { name: "Show" }));

		const chipRow = screen.getByTestId("axis-health-chips-j4");
		const igbtChip = Array.from(chipRow.querySelectorAll("span")).find((span) =>
			span.textContent?.includes("IGBT 92°C"),
		);
		expect(igbtChip).toBeTruthy();
		expect(igbtChip?.className).toContain("rose");
	});

	it("surfaces motor_not_rotating reason as an info chip when drive explains why motion is blocked", async () => {
		render(
			<ControlPanel
				apiHost=""
				driveFaults={{
					servo_backend: "ethercat_rtcore",
					driver_state: "DISARMED",
					axes: [
						{
							axis: 4,
							logical_joint: 5,
							ds402_state: "SwitchOnDisabled",
							statusword: 0x0250,
							error_code: 0,
							motor_not_rotating_text: "awaiting_homing",
						},
					],
				}}
			/>,
		);

		fireEvent.click(screen.getByRole("button", { name: "Show" }));

		const chipRow = screen.getByTestId("axis-health-chips-j5");
		expect(chipRow.textContent).toContain("awaiting_homing");
	});

	it("omits the health-chip row when no extended PDO telemetry is present", async () => {
		render(
			<ControlPanel
				apiHost=""
				driveFaults={{
					servo_backend: "ethercat_rtcore",
					driver_state: "DISARMED",
					axes: [
						{
							axis: 5,
							logical_joint: 6,
							ds402_state: "SwitchOnDisabled",
							statusword: 0x1650,
							error_code: 0,
						},
					],
				}}
			/>,
		);

		fireEvent.click(screen.getByRole("button", { name: "Show" }));
		expect(screen.queryByTestId("axis-health-chips-j6")).toBeNull();
	});

	it("shows canonical truth trust source and unavailable reason in joint commissioning", async () => {
		render(
			<ControlPanel
				apiHost=""
				driveFaults={{
					servo_backend: "ethercat_rtcore",
					driver_state: "DISARMED",
					axes: [
						{
							axis: 0,
							logical_joint: 1,
							ds402_state: "SwitchOnDisabled",
							statusword: 0x1650,
							error_code: 0,
							drive_native_truth_valid: true,
							drive_native_truth_verification_source: "persisted_home_anchor_agreement",
							coordinate_system_valid: true,
						},
						{
							axis: 1,
							logical_joint: 2,
							ds402_state: "SwitchOnDisabled",
							statusword: 0x1650,
							error_code: 0,
							drive_native_truth_valid: false,
							drive_native_truth_reason: "multi_turn_feedback_invalid",
							coordinate_system_valid: false,
						},
					],
				}}
			/>,
		);

		fireEvent.click(screen.getByRole("button", { name: "Show" }));
		expect(screen.getByText(/Canonical truth trust: persisted_home_anchor_agreement/i)).toBeTruthy();
		expect(screen.getByText(/Canonical truth unavailable: multi_turn_feedback_invalid/i)).toBeTruthy();
	});

	it("shows advisory canonical truth mismatch as a trust warning", async () => {
		render(
			<ControlPanel
				apiHost=""
				driveFaults={{
					servo_backend: "ethercat_rtcore",
					driver_state: "ACTIVE",
					op_enabled_axes: 0x3f,
					axes: [
						{
							axis: 0,
							logical_joint: 1,
							ds402_state: "OperationEnabled",
							statusword: 0x9637,
							error_code: 0,
							drive_native_truth_valid: false,
							drive_native_truth_reason: "command_frame_roundtrip_mismatch",
							coordinate_system_valid: false,
						},
					],
				}}
			/>,
		);

		fireEvent.click(screen.getByRole("button", { name: "Show" }));
		expect(screen.getByText(/Canonical truth trust warning: command_frame_roundtrip_mismatch/i)).toBeTruthy();
		expect(screen.queryByText(/Canonical truth unavailable: command_frame_roundtrip_mismatch/i)).toBeNull();
		await waitFor(() => {
			const firstJogButton = screen.getAllByRole("button", { name: "+1°" })[0] as HTMLButtonElement;
			expect(firstJogButton.disabled).toBe(false);
		});
	});

	it("blocks drive-home buttons while a native-home transaction is still active", async () => {
		render(
			<ControlPanel
				apiHost=""
				driveFaults={{
					servo_backend: "ethercat_rtcore",
					driver_state: "DISARMED",
					axis_enable_mask: 0x0,
					native_home_active_axis_mask: 0x2,
					axes: [
						{
							axis: 1,
							logical_joint: 2,
							ds402_state: "SwitchOnDisabled",
							statusword: 0x1650,
							error_code: 0,
							native_home_state: 0,
							native_home_state_name: "idle",
							native_home_active: true,
							native_home_last_abort_code: 0,
							native_home_last_abort_code_hex: "0x00000000",
						},
					],
				}}
			/>,
		);

		fireEvent.click(screen.getByRole("button", { name: "Show" }));
		expect(screen.getByText(/Drive-native home still running for J2/i)).toBeTruthy();
		expect(screen.getByText(/Drive Home requested/i)).toBeTruthy();
		for (const button of screen.getAllByRole("button", { name: "Drive Home" })) {
			expect((button as HTMLButtonElement).disabled).toBe(true);
		}
	});

	it("shows a warning when native-home verification is still pending", async () => {
		vi.spyOn(window, "confirm").mockReturnValue(true);
		vi.stubGlobal("fetch", vi.fn(async (input: string | URL | Request, init?: RequestInit) => {
			const url = typeof input === "string"
				? input
				: input instanceof URL
					? input.toString()
					: input.url;
			const method = init?.method ?? "GET";
			const parsed = new URL(url, "http://localhost");
			if (parsed.pathname === "/info/joints" || parsed.pathname === "/info/joints-detailed") {
				return mockJsonResponse({
					arm_deg: [0, 0, 0, 0, 0, 0],
					arm_display_deg: [0, 0, 0, 0, 0, 0],
					gripper_deg: 0,
					read_source: "live_feedback",
				});
			}
			if (parsed.pathname === "/control/motion-status") {
				return mockJsonResponse({
					status: "ok",
					state: "idle",
					safe_for_power_transition: true,
					power_transition_blockers: [],
					power_transition_blocker_details: [],
					execution: {
						state_name: "idle",
						active_mode_name: "idle",
						controller_thread_running: false,
						rtcore_status_present: true,
						queue_depth: 0,
						queue_capacity: 4096,
						motion_done: true,
						stale_command: false,
						safe_for_power_transition: true,
						power_transition_blockers: [],
						power_transition_blocker_details: [],
					},
				});
			}
			if (parsed.pathname === "/control/home-joint-native" && method === "POST") {
				return mockJsonResponse({
					status: "ok",
					accepted: true,
					verified: false,
					timed_out: true,
					code: "NATIVE_HOME_PENDING_VERIFICATION",
					message: "Drive-native commissioning home was requested, but verification is still pending.",
					joint: 1,
					native_home_state: 1,
					native_home_state_name: "requested",
				});
			}
			return mockJsonResponse({});
		}));

		render(
			<ControlPanel
				apiHost=""
				driveFaults={{
					servo_backend: "ethercat_rtcore",
					driver_state: "DISARMED",
					axes: [],
				}}
			/>,
		);

		fireEvent.click(screen.getByRole("button", { name: "Show" }));
		await waitFor(() => {
			expect((screen.getAllByRole("button", { name: "Drive Home" })[0] as HTMLButtonElement).disabled).toBe(false);
		});
		fireEvent.click(screen.getAllByRole("button", { name: "Drive Home" })[0]);

		await waitFor(() => {
			expect(screen.getByText(/verification is still pending/i)).toBeTruthy();
		});
		expect(screen.queryByText(/Failed to request drive-native home/i)).toBeNull();
	});

	it("uses wait-for-idle semantics for drive power-down", async () => {
		vi.spyOn(window, "confirm").mockReturnValue(true);

		render(
			<ControlPanel
				apiHost=""
				driveFaults={{
					servo_backend: "ethercat_rtcore",
					driver_state: "ACTIVE",
					axes: [],
				}}
				motionStatus={{
					status: "ok",
					state: "idle",
					safe_for_power_transition: true,
					power_transition_blockers: [],
					power_transition_blocker_details: [],
					execution: {
						state_name: "idle",
						active_mode_name: "idle",
						queue_depth: 0,
						queue_capacity: 4096,
						motion_done: true,
						stale_command: false,
						safe_for_power_transition: true,
						power_transition_blockers: [],
						power_transition_blocker_details: [],
					},
				}}
			/>,
		);

		fireEvent.click(screen.getByRole("button", { name: "Power Down Drives" }));

		await waitFor(() => {
			expect(
				fetchCalls.some(
					(call) =>
						call.method === "POST"
						&& call.pathname === "/control/power-down"
						&& JSON.stringify(call.body) === JSON.stringify({ wait_for_idle: true }),
				),
			).toBe(true);
		});
	});
});
