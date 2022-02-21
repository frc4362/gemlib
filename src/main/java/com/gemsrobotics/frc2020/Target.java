package com.gemsrobotics.frc2020;

import com.gemsrobotics.lib.math.se2.Translation;

import java.util.Optional;

public interface Target {
	double getAge();
	double getDriveDistanceSinceCapture();
	Translation getVehicleToOuterGoal();
	Optional<Translation> getVehicleToInnerGoal();
	default boolean isClose() {
		return false;
	}
	default Translation getOptimalGoal() {
		return getVehicleToInnerGoal().orElseGet(this::getVehicleToOuterGoal);
	}
}
