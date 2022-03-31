package com.gemsrobotics.frc2022.autonomous;

import com.gemsrobotics.frc2022.subsystems.Chassis;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.List;

public class Flashy5Ball extends SequentialCommandGroup {
	public Flashy5Ball() {
		final var chassis = Chassis.getInstance();

		final var traj1 = chassis.getGeneratedWPITrajectory(List.of(
				new RigidTransform()
		));
	}
}
