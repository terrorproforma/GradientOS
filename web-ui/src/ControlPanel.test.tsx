import { cleanup, fireEvent, render, screen, waitFor } from "@testing-library/react";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { ControlPanel } from "./ControlPanel";

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

			if (parsed.pathname === "/info/joints") {
				return mockJsonResponse({
					arm_deg: [0, 0, 0, 0, 0, 0],
					gripper_deg: 0,
				});
			}
			if (parsed.pathname === "/control/motion-status") {
				return mockJsonResponse({
					status: "ok",
					state: "idle",
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
		vi.unstubAllGlobals();
	});

	it("does not open a jog session while armed but idle", async () => {
		render(<ControlPanel apiHost="" />);

		fireEvent.click(screen.getByRole("button", { name: "Arm" }));
		await sleep(400);

		const jogPosts = fetchCalls.filter((call) =>
			call.method === "POST" && call.pathname.startsWith("/control/jog/session/"),
		);
		expect(jogPosts).toEqual([]);
	});

	it("starts on pointer down and stops on final release", async () => {
		render(<ControlPanel apiHost="" />);

		fireEvent.click(screen.getByRole("button", { name: "Arm" }));
		await waitFor(() => {
			expect(screen.getByRole("button", { name: "Disarm" })).toBeTruthy();
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
});
