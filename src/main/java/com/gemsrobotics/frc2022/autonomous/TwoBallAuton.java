package com.gemsrobotics.frc2022.autonomous;

import com.gemsrobotics.frc2022.commands.*;
import com.gemsrobotics.frc2022.subsystems.Chassis;
import com.gemsrobotics.frc2022.subsystems.OurPicoSensor;
import com.gemsrobotics.frc2022.subsystems.Superstructure;
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
				new RigidTransform(new Translation(Units.inches2Meters(65), Units.inches2Meters(-14)), Rotation.degrees(-20))
		));

		final var pivot = new RigidTransform(new Translation(Units.inches2Meters(65), Units.inches2Meters(-14)), Rotation.degrees(-80));
		final var endPivot = pivot.transformBy(RigidTransform.fromTranslation(new Translation(Units.inches2Meters(36.0), 0)));
		final var trajectory2 = chassis.getGeneratedWPITrajectory(List.of(
			pivot,
			endPivot
		));

		final var nextPivot = new RigidTransform(endPivot.getTranslation(), Rotation.degrees(100));
		final var ending = nextPivot.transformBy(new RigidTransform(new Translation(Units.inches2Meters(127.5), Units.inches2Meters(50)), Rotation.degrees(30)));
		final var trajectory3 = chassis.getGeneratedWPITrajectory(List.of(
			nextPivot,
			ending
		));

		final var G = ending.transformBy(new RigidTransform(new Translation(Units.inches2Meters(-40), Units.inches2Meters(60)), Rotation.degrees(-60)));

		final var trajectory4 = chassis.getReversedTrajectory(List.of(
			ending,
			G,
			G.transformBy(new RigidTransform(new Translation(Units.inches2Meters(-20), Units.inches2Meters(8)), Rotation.degrees(-10)))));

//		final var trajectory2 = chassis.getGeneratedWPITrajectory(List.of(
//				new RigidTransform(new Translation(Units.inches2Meters(40), 0.0), Rotation.degrees(20.0)),
//				new RigidTransform(new Translation(Units.inches2Meters(40) + Units.inches2Meters(20), -10), Rotation.degrees(20.0))
//		));

		addCommands(
				new ResetOdometerCommand(chassis, trajectory1),
				new InstantCommand(() -> OurPicoSensor.getInstance().setFilterDisabled()),
				new SetTurretGuess(Rotation.degrees(-150)),
				new PrepareShotCommand(true),
				new ParallelCommandGroup(
						new IntakeCommand(2, 15.0),
						new SequentialCommandGroup(
								new WaitCommand(0.25),
								new GemRamseteCommand(trajectory1)
						)
				),
				new ShootAllBalls(),
				new PrepareShotCommand(false),
				new TurnToHeading(Rotation.degrees(-80)),
				new ParallelCommandGroup(
						new IntakeCommand(1, 10.0),
						new GemRamseteCommand(trajectory2)
				),
				new TurnToHeading(Rotation.degrees(100)).withTimeout(3.5),
				new ParallelCommandGroup(
					new SequentialCommandGroup(
						new WaitCommand(1.0),
						new IntakeCommand(2, 5.0)),
					new GemRamseteCommand(trajectory3)
				),
				new GemRamseteCommand(trajectory4)

				// new InstantCommand(() -> Superstructure.getInstance().setWantedState(Superstructure.WantedState.OUTTAKING)),
				// new WaitCommand(3.0),
				// new InstantCommand(() -> Superstructure.getInstance().setWantedState(Superstructure.WantedState.IDLE))
				// new InstantCommand(() -> OurPicoSensor.getInstance().setFilterDefault())
//				new GemRamseteCommand(trajectory2)
		);
	}
}
