package com.gemsrobotics.frc2019.util.command;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Represents a {@link CommandGroup} for Auton which is dependant on information
 * which is not determinable until {@link DriverStation#getGameSpecificMessage()}
 * has a value. This cannot be done in {@link CommandGroup#initialize()}
 */
public abstract class RuntimeCommandGroup extends CommandGroup {
	public abstract void init();
}
