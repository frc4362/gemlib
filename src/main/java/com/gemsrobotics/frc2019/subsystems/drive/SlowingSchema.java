package com.gemsrobotics.frc2019.subsystems.drive;

public class SlowingSchema {
	public double startRampArea, endRampArea, minimumSpeed;

	public double getRampingRange() {
		return 1.0 - minimumSpeed;
	}

	public double getDivisor() {
		return startRampArea - endRampArea;
	}
}
