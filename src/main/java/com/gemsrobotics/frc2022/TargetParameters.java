package com.gemsrobotics.frc2022;

import com.gemsrobotics.lib.math.se2.Translation;

public interface TargetParameters {
	/**
	 * @return The time in seconds since the image was captured
	 */
	double getAgeSeconds();

	/**
	 * @return The translation from the turret to the goal, in the frame of reference of the field
	 */
	Translation getCurrentTurretToGoal();
}
