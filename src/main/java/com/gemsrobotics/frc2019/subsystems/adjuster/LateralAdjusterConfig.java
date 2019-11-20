package com.gemsrobotics.frc2019.subsystems.adjuster;

import com.gemsrobotics.frc2019.util.PIDF;

@SuppressWarnings({"unused", "WeakerAccess"})
public class LateralAdjusterConfig {
	public int port, leftDist, rightDist;
	public double widthInches, nominalVolts, adjustmentThresholdRadians;
	public PIDF pidVars;
}
