package com.gemsrobotics.frc2020.autonomous;

import com.gemsrobotics.frc2020.subsystems.Chassis;
import com.gemsrobotics.lib.commands.ResetOdometerCommand;
import com.gemsrobotics.lib.commands.TrackTrajectoryCommand;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.utils.Units;
import edu.wpi.first.wpilibj.command.CommandGroup;

import java.util.Collections;
import java.util.List;

public class Bounce extends CommandGroup {
	public Bounce() {
		final var c = Chassis.getInstance();
		final var traj1 = c.getTrajectoryGenerator().generateTrajectory(
				false,
				false,
				List.of(
						new RigidTransform(new Translation(Units.inches2Meters(55.0), Units.inches2Meters(90.0)), Rotation.identity()),
//						new RigidTransform(new Translation(Units.inches2Meters(90.0), Units.inches2Meters(120.0)), Rotation.degrees(80)),
						new RigidTransform(new Translation(Units.inches2Meters(105.0), Units.inches2Meters(150.0)), Rotation.degrees(90))
				),
				Collections.emptyList()
		);
		final var traj2 = c.getTrajectoryGenerator().generateTrajectory(
				false,
				true,
				List.of(
						new RigidTransform(new Translation(Units.inches2Meters(105.0), Units.inches2Meters(130.0)), Rotation.degrees(90)),
						new RigidTransform(new Translation(Units.inches2Meters(110.0), Units.inches2Meters(90.0)), Rotation.degrees(135)),
//						new RigidTransform(new Translation(Units.inches2Meters(120.0), Units.inches2Meters(60.0)), Rotation.degrees(135)),
						new RigidTransform(new Translation(Units.inches2Meters(150.0), Units.inches2Meters(30.0)), Rotation.degrees(135)),
						new RigidTransform(new Translation(Units.inches2Meters(200.0), Units.inches2Meters(60.0)), Rotation.degrees(-90)),
						new RigidTransform(new Translation(Units.inches2Meters(200.0), Units.inches2Meters(150.0)), Rotation.degrees(-80))
				),
				Collections.emptyList()
		);
		final var traj3 = c.getTrajectoryGenerator().generateTrajectory(
				false,
				false,
				List.of(
						new RigidTransform(new Translation(Units.inches2Meters(180.0), Units.inches2Meters(120.0)), Rotation.degrees(-90)),
						new RigidTransform(new Translation(Units.inches2Meters(225.0), Units.inches2Meters(30.0)), Rotation.degrees(0)),
						new RigidTransform(new Translation(Units.inches2Meters(280.0), Units.inches2Meters(150.0)), Rotation.degrees(90))
				),
				Collections.emptyList()
		);
		final var traj4 = c.getTrajectoryGenerator().generateTrajectory(
				false,
				true,
				List.of(
						new RigidTransform(new Translation(Units.inches2Meters(280.0), Units.inches2Meters(100.0)), Rotation.degrees(135)),
						new RigidTransform(new Translation(Units.inches2Meters(320.0), Units.inches2Meters(80.0)), Rotation.degrees(180))
				),
				Collections.emptyList()
		);
		addSequential(new ResetOdometerCommand(c, traj1.getTrajectory().getTrajectory()));
		addSequential(new TrackTrajectoryCommand(c, traj1));
		addSequential(new TrackTrajectoryCommand(c, traj2));
		addSequential(new TrackTrajectoryCommand(c, traj3));
		addSequential(new TrackTrajectoryCommand(c, traj4));
	}
}
