package com.gemsrobotics.frc2022.autonomous;

import com.gemsrobotics.frc2022.commands.*;
import com.gemsrobotics.frc2022.subsystems.Chassis;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.utils.Units;
import edu.wpi.first.wpilibj2.command.*;

import java.util.List;

public class TwoBallAuton extends SequentialCommandGroup {
	public TwoBallAuton() {
		final var chassis = Chassis.getInstance();

		final var trajectory1 = chassis.getGeneratedWPITrajectory(List.of(
				new RigidTransform(new Translation(0.0, 0.0), Rotation.degrees(0.0)),
				new RigidTransform(new Translation(Units.inches2Meters(60), Units.inches2Meters(-14)), Rotation.degrees(-20))
		));

//		final var trajectory2 = chassis.getGeneratedWPITrajectory(List.of(
//				new RigidTransform(new Translation(Units.inches2Meters(40), 0.0), Rotation.degrees(20.0)),
//				new RigidTransform(new Translation(Units.inches2Meters(40) + Units.inches2Meters(20), -10), Rotation.degrees(20.0))
//		));

		addCommands(
				new ResetOdometerCommand(chassis, trajectory1),
				new SetTurretGuess(Rotation.degrees(-150)),
				new ParallelCommandGroup(
						new IntakeCommand(2, 15.0),
						new SequentialCommandGroup(
								new WaitCommand(0.25),
								new GemRamseteCommand(trajectory1)
						)
				),
				new ShootAllBalls()
//				new GemRamseteCommand(trajectory2)
		);
	}
}
