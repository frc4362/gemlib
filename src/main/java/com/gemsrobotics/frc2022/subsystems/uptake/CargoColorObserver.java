package com.gemsrobotics.frc2022.subsystems.uptake;

import edu.wpi.first.wpilibj.DriverStation;

public interface CargoColorObserver {
	DriverStation.Alliance getCargoAlliance();
	boolean isCargoOurs();
}
