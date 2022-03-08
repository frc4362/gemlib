package com.gemsrobotics.frc2022.autonomous;

import com.gemsrobotics.frc2022.subsystems.Chassis;
import com.gemsrobotics.lib.commands.ResetOdometerCommand;
import com.gemsrobotics.lib.commands.TrackTrajectoryCommand;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;
import edu.wpi.first.wpilibj.command.CommandGroup;

import java.util.Collections;
import java.util.List;

public class TestAuton extends CommandGroup {
	public TestAuton() {
		final var chassis = Chassis.getInstance();

		final var trajectory = chassis.getTrajectoryGenerator().generateTrajectory(
				false,
				false,
				List.of(
						new RigidTransform(new Translation(0.0, 0.0), Rotation.identity()),
						new RigidTransform(new Translation(3.0, 2.0), Rotation.degrees(90))
				),
				Collections.emptyList()
		);

		addSequential(new ResetOdometerCommand(chassis, trajectory.getTrajectory().getTrajectory()));
		addSequential(new TrackTrajectoryCommand(chassis, trajectory));
	}
}
