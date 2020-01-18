package com.gemsrobotics.frc2020.subsystems;

import com.gemsrobotics.lib.math.se2.Translation;

public final class GoalInfo {
	public final double timestamp, distance;
	public final Translation goalPosition;

	public GoalInfo(final double timestamp, final double distance, final Translation field2Goal) {
		this.timestamp = timestamp;
		this.distance = distance;
		this.goalPosition = field2Goal;
	}
}
