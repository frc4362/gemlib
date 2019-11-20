package com.gemsrobotics.frc2019.subsystems.lift;

import com.gemsrobotics.frc2019.util.PIDF;

@SuppressWarnings("WeakerAccess")
public class LiftConfig {
	int portMaster, portSlave;
	double rotationsTop, rotationsBottom;
	double cameraBlockedTop, cameraBlockedBottom;
	PIDF pidVars;

	public double totalRotations() {
		return rotationsTop - rotationsBottom;
	}
}
