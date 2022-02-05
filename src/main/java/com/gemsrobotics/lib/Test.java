package com.gemsrobotics.lib;

import com.gemsrobotics.frc2020.subsystems.RobotState;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;

public class Test {
	public static void main(String[] args) {
		final var fieldToOuterGoal = new Translation(40, 0);
		final var turretPose = new RigidTransform(new Translation(20.0, 20.0), Rotation.degrees(45));
		final double vy = fieldToOuterGoal.y() - turretPose.getTranslation().y();
		final double vx = fieldToOuterGoal.x() - turretPose.getTranslation().x();
		final double angleToAim = Math.atan2(vy, vx);
		final double range = Math.sqrt(vy * vy + vx * vx);

//		final var diff = fieldToOuterGoal.difference(turretPose.getTranslation());
//		var angleToAim = diff.direction().getRadians();
//		var range = diff.norm();

//		System.out.println("angle: " + angleToAim);
//		System.out.println("range: " + range);

		System.out.println(Translation.fromPolar(Rotation.radians(angleToAim), range));
	}
}
