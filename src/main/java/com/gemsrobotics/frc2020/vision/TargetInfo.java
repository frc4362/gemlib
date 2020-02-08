package com.gemsrobotics.frc2020.vision;

import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;

public class TargetInfo {
	public final double timestamp;
	public final RigidTransform cameraToTarget;

	public TargetInfo(final double timestamp, final RigidTransform cameraToTarget) {
		this.timestamp = timestamp;
		this.cameraToTarget = cameraToTarget;
	}
}
