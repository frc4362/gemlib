package com.gemsrobotics.frc2020.autonomous;

import com.gemsrobotics.frc2020.commands.*;
import com.gemsrobotics.frc2020.subsystems.Chassis;
import com.gemsrobotics.frc2020.subsystems.Superstructure;
import com.gemsrobotics.lib.commands.ResetOdometerCommand;
import com.gemsrobotics.lib.commands.WaitCommand;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.utils.Units;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class SixBallAuton extends CommandGroup {
	public SixBallAuton(final Rotation guess) {
		addSequential(new SetTurretGuess(Superstructure.getInstance(), guess));
		addSequential(new SetWantedStateCommand(Superstructure.getInstance(), Superstructure.WantedState.SHOOTING));
		addSequential(new WaitForStateCommand(Superstructure.getInstance(), Superstructure.SystemState.SHOOTING), 2.0);
		addSequential(new WaitCommand(2.5));
		addSequential(new SetWantedStateCommand(Superstructure.getInstance(), Superstructure.WantedState.INTAKING));
		addSequential(new DriveStraightCommand(Chassis.getInstance(), 0.1, Units.feet2Meters(12.0)));
		addSequential(new SetWantedStateCommand(Superstructure.getInstance(), Superstructure.WantedState.SHOOTING));
		addSequential(new WaitCommand(5.0));
		addSequential(new SetWantedStateCommand(Superstructure.getInstance(), Superstructure.WantedState.IDLE));
	}
}
