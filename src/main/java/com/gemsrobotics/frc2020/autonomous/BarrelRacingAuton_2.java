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

public class BarrelRacingAuton_2 extends CommandGroup {
	public BarrelRacingAuton_2() {
		final var c = Chassis.getInstance();
		final var traj = c.getTrajectoryGenerator().generateTrajectory(
				false,
				false,
//				List.of(
//						RigidTransform.identity(),
//						new RigidTransform(new Translation(Units.feet2Meters(12.0), 0.0), Rotation.identity())
//				),
				List.of(
						new RigidTransform(new Translation(Units.inches2Meters(60.0), Units.inches2Meters(90.0)), Rotation.identity()),
						new RigidTransform(new Translation(Units.inches2Meters(180.0), Units.inches2Meters(60)), Rotation.degrees(-90)),
						new RigidTransform(new Translation(Units.inches2Meters(120), Units.inches2Meters(60.0)), Rotation.degrees(90)),
						new RigidTransform(new Translation(Units.inches2Meters(240.0), Units.inches2Meters(150.0)), Rotation.degrees(180)),
						new RigidTransform(new Translation(Units.inches2Meters(300.0), Units.inches2Meters(30.0)), Rotation.degrees(0)),
						new RigidTransform(new Translation(Units.inches2Meters(60.0), Units.inches2Meters(90.0)), Rotation.degrees(180))
//						new RigidTransform(new Translation(Units.inches2Meters(130.0), Units.inches2Meters(30.0)), Rotation.degrees(-180)),
//						new RigidTransform(new Translation(Units.inches2Meters(140.0), Units.inches2Meters(60.0)), Rotation.degrees(90)),
//						new RigidTransform(new Translation(Units.inches2Meters(170.0), Units.inches2Meters(90.0)), Rotation.degrees(0)),
//						new RigidTransform(new Translation(Units.inches2Meters(200.0), Units.inches2Meters(90.0)), Rotation.degrees(0))
				),
				Collections.emptyList()
		);
		addSequential(new ResetOdometerCommand(c, traj.getTrajectory().getTrajectory()));
		addSequential(new TrackTrajectoryCommand(c, traj));
	}
}
