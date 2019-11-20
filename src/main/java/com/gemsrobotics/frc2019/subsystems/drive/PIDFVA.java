package com.gemsrobotics.frc2019.subsystems.drive;

import jaci.pathfinder.followers.DistanceFollower;

@SuppressWarnings({"unused", "WeakerAccess"})
public class PIDFVA {
	public double kP, kI, kD, kF, kV, kA;

	public void configure(final DistanceFollower follower) {
		follower.configurePIDVA(kP, kI, kD, kV, kA);
	}
}
