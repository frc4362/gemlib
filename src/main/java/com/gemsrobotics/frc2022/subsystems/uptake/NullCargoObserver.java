package com.gemsrobotics.frc2022.subsystems.uptake;

import edu.wpi.first.wpilibj.DriverStation;

public class NullCargoObserver implements CargoColorObserver {
	@Override
	public DriverStation.Alliance getCargoAlliance() {
		return DriverStation.Alliance.Invalid;
	}

	@Override
	public boolean isCargoOurs() {
		return true;
	}
}
