package com.gemsrobotics.frc2020;

import com.gemsrobotics.lib.math.se2.Translation;

import java.util.Optional;

public interface Target {
	double getAge();
	double getDriveDistanceSinceCapture();
	Translation getFieldToOuterGoal();
	Optional<Translation> getFieldToInnerGoal();
	default boolean isClose() {
		return false;
	}
	default Translation getOptimalGoal() {
		return getFieldToInnerGoal().orElseGet(this::getFieldToOuterGoal);
	}
}
