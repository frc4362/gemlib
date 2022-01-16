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

public class Slalom extends CommandGroup {
	public Slalom() {
		final var c = Chassis.getInstance();
		final var traj = c.getTrajectoryGenerator().generateTrajectory(
				false,
				false,
//				List.of(
//						RigidTransform.identity(),
//						new RigidTransform(new Translation(Units.feet2Meters(12.0), 0.0), Rotation.identity())
//				),
				List.of(
						new RigidTransform(new Translation(Units.inches2Meters(60.0), Units.inches2Meters(30.0)), Rotation.identity()),
						new RigidTransform(new Translation(Units.inches2Meters(80.0), Units.inches2Meters(30.0)), Rotation.identity()),
						new RigidTransform(new Translation(Units.inches2Meters(150.0), Units.inches2Meters(130.0)), Rotation.identity()),
						new RigidTransform(new Translation(Units.inches2Meters(220.0), Units.inches2Meters(130.0)), Rotation.identity()),
						new RigidTransform(new Translation(Units.inches2Meters(285.0), Units.inches2Meters(50.0)), Rotation.degrees(-90)),
						new RigidTransform(new Translation(Units.inches2Meters(300.0), Units.inches2Meters(20.0)), Rotation.identity()),
						new RigidTransform(new Translation(Units.inches2Meters(340.0), Units.inches2Meters(60.0)), Rotation.degrees(90)),
						new RigidTransform(new Translation(Units.inches2Meters(300.0), Units.inches2Meters(110.0)), Rotation.degrees(-180)),
						new RigidTransform(new Translation(Units.inches2Meters(220.0), Units.inches2Meters(20.0)), Rotation.degrees(-180)),
						new RigidTransform(new Translation(Units.inches2Meters(130.0), Units.inches2Meters(20.0)), Rotation.degrees(-180)),
						new RigidTransform(new Translation(Units.inches2Meters(40.0), Units.inches2Meters(100.0)), Rotation.degrees(-180))
				),
				Collections.emptyList()
		);
		addSequential(new ResetOdometerCommand(c, traj.getTrajectory().getTrajectory()));
		addSequential(new TrackTrajectoryCommand(c, traj));
	}
}
