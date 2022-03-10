package com.gemsrobotics.frc2022.autonomous;

import com.gemsrobotics.frc2022.commands.ResetOdometerCommand;
import com.gemsrobotics.frc2022.subsystems.Chassis;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.utils.Units;
import edu.wpi.first.wpilibj2.command.*;

import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class FiveBallAuton extends SequentialCommandGroup {
	public FiveBallAuton() {
		final var chassis = Chassis.getInstance();
		final var origin = RigidTransform.fromRotation(Rotation.degrees(-62.5));
		final var secondPoint = origin.transformBy(new RigidTransform(new Translation(Units.inches2Meters(24.0), 0.0), Rotation.identity()));
//		final var trajectoryPoints1 = Stream.of(
//				new RigidTransform(new Translation(0.0, 0.0), Rotation.identity()),
//				new RigidTransform(new Translation(1.7, -2.0), Rotation.degrees(-120)),
//				new RigidTransform(new Translation(0.5, -3.0), Rotation.degrees(-150)))
//				new RigidTransform(new Translation(Units.inches2Meters(36.0), Units.inches2Meters(-50)), Rotation.degrees(-90)))
//			.map(rt -> rt.inFrameOfReferenceOf(origin))
//			.collect(Collectors.toList());

		final var trajectory1 = chassis.getGeneratedWPITrajectory(List.of(
				new RigidTransform(new Translation(0.0, 0.0), Rotation.degrees(0.0)),
				new RigidTransform(new Translation(Units.inches2Meters(42), Units.inches2Meters(0.0)), Rotation.identity())
		));

		final var trajectory2 = chassis.getReversedTrajectory(List.of(
				new RigidTransform(new Translation(Units.inches2Meters(42), Units.inches2Meters(0.0)), Rotation.identity()),
				new RigidTransform(new Translation(Units.inches2Meters(12), Units.inches2Meters(20.0)), Rotation.degrees(-80))
		));

		final var trajectory4 = chassis.getGeneratedWPITrajectory(List.of(
				new RigidTransform(new Translation(Units.inches2Meters(5.0), Units.inches2Meters(40)), Rotation.degrees(-80)),
				new RigidTransform(new Translation(Units.inches2Meters(-5.0), Units.inches2Meters(-80)), Rotation.degrees(-87))
		));

		final var trajectory5 = chassis.getGeneratedWPITrajectory(List.of(
				new RigidTransform(new Translation(Units.inches2Meters(-5), Units.inches2Meters(-80)), Rotation.degrees(-87)),
				new RigidTransform(new Translation(Units.inches2Meters(15), Units.inches2Meters(-80 - (13.0 * 12))), Rotation.degrees(-60))
		));

		final var trajectory6 = chassis.getReversedTrajectory(List.of(
				new RigidTransform(new Translation(Units.inches2Meters(15), Units.inches2Meters(-80 - (13.0 * 12))), Rotation.degrees(-60)),
				new RigidTransform(new Translation(Units.inches2Meters(-30), Units.inches2Meters(-120)), Rotation.degrees(-100))
		));

		addCommands(
				new ResetOdometerCommand(chassis, trajectory1),
				new SetTurretGuess(Rotation.degrees(150)),
				new ParallelCommandGroup(
						new IntakeCommand(2, 45.0),
						new SequentialCommandGroup(
							new WaitCommand(0.0),
							new GemRamseteCommand(trajectory1))
				),
				new ShootAllBalls(),
				new GemRamseteCommand(trajectory2),
				new ParallelCommandGroup(
						new IntakeCommand(1, 45.0),
						new GemRamseteCommand(trajectory4)
				),
				new ShootAllBalls(),
				new ParallelDeadlineGroup(
					new SequentialCommandGroup(
						new GemRamseteCommand(trajectory5),
						new WaitCommand(0.2),
						new GemRamseteCommand(trajectory6)
					),
					new SequentialCommandGroup(
						new WaitCommand(1.0),
						new IntakeCommand(2, 45.0)
					)
				),
				new ShootAllBalls());
	}
}
