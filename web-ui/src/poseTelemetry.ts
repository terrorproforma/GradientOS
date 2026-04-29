export type PoseTelemetrySnapshot = {
	joints?: number[] | null;
	display_joints?: number[] | null;
};

export function preferredDisplayPoseJoints(
	latest: PoseTelemetrySnapshot | null | undefined,
): number[] | null {
	if (Array.isArray(latest?.display_joints) && latest.display_joints.length > 0) {
		return latest.display_joints;
	}
	if (Array.isArray(latest?.joints) && latest.joints.length > 0) {
		return latest.joints;
	}
	return null;
}

export function preferredLiveVisualizerPoseJoints(
	latest: PoseTelemetrySnapshot | null | undefined,
): number[] | null {
	if (Array.isArray(latest?.joints) && latest.joints.length > 0) {
		return latest.joints;
	}
	if (Array.isArray(latest?.display_joints) && latest.display_joints.length > 0) {
		return latest.display_joints;
	}
	return null;
}
