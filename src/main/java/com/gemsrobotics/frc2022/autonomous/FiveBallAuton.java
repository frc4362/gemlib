package com.gemsrobotics.frc2022.autonomous;

import com.gemsrobotics.frc2022.commands.*;
import com.gemsrobotics.frc2022.subsystems.Chassis;
import com.gemsrobotics.frc2022.subsystems.Superstructure;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.utils.Units;
import edu.wpi.first.wpilibj2.command.*;

import java.util.List;

public class FiveBallAuton extends SequentialCommandGroup {
	public FiveBallAuton() {
		final var chassis = Chassis.getInstance();

		final var trajectory1 = chassis.getGeneratedWPITrajectory(List.of(
				new RigidTransform(new Translation(0.0, 0.0), Rotation.degrees(0.0)),
				new RigidTransform(new Translation(Units.inches2Meters(42), Units.inches2Meters(0.0)), Rotation.identity())
		));

		final var trajectory2 = chassis.getReversedTrajectory(List.of(
				new RigidTransform(new Translation(Units.inches2Meters(42), Units.inches2Meters(0.0)), Rotation.identity()),
				new RigidTransform(new Translation(Units.inches2Meters(12), Units.inches2Meters(20.0)), Rotation.degrees(-80))
		));

		final var trajectory4 = chassis.getGeneratedWPITrajectory(List.of(
				new RigidTransform(new Translation(Units.inches2Meters(42), Units.inches2Meters(0.0)), Rotation.degrees(-110)),
				new RigidTransform(new Translation(Units.inches2Meters(5.0), Units.inches2Meters(-95)), Rotation.degrees(-110))
		));

		final var pickupPose = new RigidTransform(new Translation(Units.inches2Meters(25.0), Units.inches2Meters(-78 - (13.5 * 12))), Rotation.degrees(-55));
		final var newPickupPose = pickupPose.transformBy(new RigidTransform(new Translation(Units.inches2Meters(-5), Units.inches2Meters(4)), Rotation.degrees(2)));

		final var trajectory5 = chassis.getGeneratedWPITrajectory(List.of(
				new RigidTransform(new Translation(Units.inches2Meters(5.0), Units.inches2Meters(-95)), Rotation.degrees(-110)),
				newPickupPose
		));

		final var trajectory6_5 = chassis.getReversedTrajectory(List.of(
			newPickupPose,
			newPickupPose.transformBy(RigidTransform.fromTranslation(new Translation(Units.inches2Meters(-8), 0.0)))
		));

		final var trajectory6 = chassis.getReversedTrajectory(List.of(
				newPickupPose,
				new RigidTransform(new Translation(Units.inches2Meters(20.0), Units.inches2Meters(-110)), Rotation.degrees(-90)),
				new RigidTransform(new Translation(Units.inches2Meters(40.0), Units.inches2Meters(-40)), Rotation.degrees(-90))
		));

		addCommands(
				new ResetOdometerCommand(chassis, trajectory1),
				new SetTurretLockedCommand(true),
				new SetTurretGuess(Rotation.degrees(-90)),
				new ParallelCommandGroup(
						new IntakeCommand(2, 2.0),
						new SequentialCommandGroup(
							new WaitCommand(0.0),
							new GemRamseteCommand(trajectory1))
				),
//				new ShootAllBalls(),
				new TurnToHeading(Rotation.degrees(-110)),
//				new GemRamseteCommand(trajectory2),
				new PrepareShotCommand(true),
				new SetTurretLockedCommand(false),
				new InstantCommand(() -> Superstructure.getInstance().setWantedState(Superstructure.WantedState.INTAKING)),
				new GemRamseteCommand(trajectory4),
				new ShootAllBalls(true, 1.3),
				new SetTurretLockedCommand(true),
				new ParallelDeadlineGroup(
					new SequentialCommandGroup(
						new ParallelCommandGroup(
								new WaitCommand(0.5),
								new PrepareShotCommand(false)
						),
						new GemRamseteCommand(trajectory5),
						new WaitCommand(0.25),
						new GemRamseteCommand(trajectory6_5),
						new WaitCommand(0.7),
						new PrepareShotCommand(true),
						new ParallelCommandGroup(
								new SequentialCommandGroup(
										new WaitCommand(1.5),
										new SetTurretLockedCommand(false)
								),
								new GemRamseteCommand(trajectory6)
						)
					),
					new SequentialCommandGroup(
						new WaitCommand(1.),
						new IntakeCommand(2, 15.0)
					)
				),
				new ShootAllBalls(),
				new TurnToHeading(Rotation.degrees(90))
		);
	}
}
