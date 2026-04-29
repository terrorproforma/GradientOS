import { describe, expect, it } from "vitest";
import { preferredDisplayPoseJoints, preferredLiveVisualizerPoseJoints } from "./poseTelemetry";

describe("preferredDisplayPoseJoints", () => {
	it("prefers operator display joints when both streams are present", () => {
		expect(
			preferredDisplayPoseJoints({
				joints: [-628, -113, -62],
				display_joints: [0.01, 0.02, 0.03],
			}),
		).toEqual([0.01, 0.02, 0.03]);
	});

	it("falls back to raw joints when display joints are absent", () => {
		expect(
			preferredDisplayPoseJoints({
				joints: [0.1, 0.2, 0.3],
			}),
		).toEqual([0.1, 0.2, 0.3]);
	});

	it("returns null when neither pose stream is available", () => {
		expect(preferredDisplayPoseJoints(null)).toBeNull();
		expect(preferredDisplayPoseJoints({})).toBeNull();
	});
});

describe("preferredLiveVisualizerPoseJoints", () => {
	it("prefers live joints over display joints for physical visual tracking", () => {
		expect(
			preferredLiveVisualizerPoseJoints({
				joints: [-628, -113, -62],
				display_joints: [0.01, 0.02, 0.03],
			}),
		).toEqual([-628, -113, -62]);
	});

	it("falls back to display joints when live joints are absent", () => {
		expect(
			preferredLiveVisualizerPoseJoints({
				display_joints: [0.01, 0.02, 0.03],
			}),
		).toEqual([0.01, 0.02, 0.03]);
	});

	it("returns null when neither pose stream is available", () => {
		expect(preferredLiveVisualizerPoseJoints(null)).toBeNull();
		expect(preferredLiveVisualizerPoseJoints({})).toBeNull();
	});
});
