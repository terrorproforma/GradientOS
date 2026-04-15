import { cleanup, fireEvent, render, screen, waitFor } from "@testing-library/react";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { ControlPanel, ControlPanelRuntimeHeader } from "./ControlPanel";

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
		expect(jogPosts[jogPosts.length - 1]?.pathname).toBe("/control/jog/session/stop");
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

	it("does not show success when statusword fallback masks a reported native-home failure", async () => {
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
		expect(screen.getByText(/Drive Home verification conflicted/i)).toBeTruthy();
		expect(screen.getByText(/reported failed 0x06010002/i)).toBeTruthy();
		expect(screen.queryByText(/Drive Home succeeded/i)).toBeNull();
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
