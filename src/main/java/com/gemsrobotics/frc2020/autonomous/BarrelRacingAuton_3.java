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

public class BarrelRacingAuton_3 extends CommandGroup {
	public BarrelRacingAuton_3() {
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
						new RigidTransform(new Translation(Units.inches2Meters(180.5), Units.inches2Meters(90.6)), Rotation.degrees(352)),
						new RigidTransform(new Translation(Units.inches2Meters(200.0), Units.inches2Meters(60.6)), Rotation.degrees(267)),
						new RigidTransform(new Translation(Units.inches2Meters(150.5), Units.inches2Meters(25.5)), Rotation.degrees(180)),
						new RigidTransform(new Translation(Units.inches2Meters(125.3), Units.inches2Meters(60.6)), Rotation.degrees(93)),
						new RigidTransform(new Translation(Units.inches2Meters(150.5), Units.inches2Meters(90.6)), Rotation.degrees(9)),
						new RigidTransform(new Translation(Units.inches2Meters(270.4), Units.inches2Meters(90.1)), Rotation.degrees(8)),
						new RigidTransform(new Translation(Units.inches2Meters(290.0), Units.inches2Meters(120.0)), Rotation.degrees(88)),
						new RigidTransform(new Translation(Units.inches2Meters(240.8), Units.inches2Meters(153.0)), Rotation.degrees(180)),
						new RigidTransform(new Translation(Units.inches2Meters(205.0), Units.inches2Meters(123.8)), Rotation.degrees(256)),
						new RigidTransform(new Translation(Units.inches2Meters(230.0), Units.inches2Meters(75.8)), Rotation.degrees(306)),
						new RigidTransform(new Translation(Units.inches2Meters(299.8), Units.inches2Meters(25.0)), Rotation.degrees(8)),
						new RigidTransform(new Translation(Units.inches2Meters(340.0), Units.inches2Meters(60.0)), Rotation.degrees(90)),
						new RigidTransform(new Translation(Units.inches2Meters(300.0), Units.inches2Meters(110.0)), Rotation.degrees(170)),
						new RigidTransform(new Translation(Units.inches2Meters(180.0), Units.inches2Meters(110.0)), Rotation.degrees(180)),
						new RigidTransform(new Translation(Units.inches2Meters(60.0), Units.inches2Meters(110.0)), Rotation.degrees(180))
				),
				Collections.emptyList()
		);
		addSequential(new ResetOdometerCommand(c, traj.getTrajectory().getTrajectory()));
		addSequential(new TrackTrajectoryCommand(c, traj));
	}
}
