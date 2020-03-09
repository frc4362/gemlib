package com.gemsrobotics.frc2020.autonomous;

import com.gemsrobotics.frc2020.commands.SetTurretGuess;
import com.gemsrobotics.frc2020.commands.SetWantedStateCommand;
import com.gemsrobotics.frc2020.commands.TimedDriveCommand;
import com.gemsrobotics.frc2020.commands.WaitForStateCommand;
import com.gemsrobotics.frc2020.subsystems.Chassis;
import com.gemsrobotics.frc2020.subsystems.Superstructure;
import com.gemsrobotics.lib.commands.ResetOdometerCommand;
import com.gemsrobotics.lib.commands.WaitCommand;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class ThreeBallAuton extends CommandGroup {
	public ThreeBallAuton(final Rotation guess) {
		addSequential(new SetTurretGuess(Superstructure.getInstance(), guess));
		addSequential(new SetWantedStateCommand(Superstructure.getInstance(), Superstructure.WantedState.SHOOTING));
		addSequential(new WaitForStateCommand(Superstructure.getInstance(), Superstructure.SystemState.SHOOTING), 4.0);
		addSequential(new WaitCommand(4.0));
		addSequential(new SetWantedStateCommand(Superstructure.getInstance(), Superstructure.WantedState.IDLE));
		addSequential(new TimedDriveCommand(Chassis.getInstance(), 0.2, 1.5));
	}
}
