package com.gemsrobotics.frc2020.vision;

import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Translation;

public final class CachedTargetInfo {
	public final double timestamp;
	public final Translation captureFieldToVehicle;

	public CachedTargetInfo(final double timestamp, final Translation captureFieldToVehicle) {
		this.timestamp = timestamp;
		this.captureFieldToVehicle = captureFieldToVehicle;
	}
}
