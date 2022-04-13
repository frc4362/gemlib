package com.gemsrobotics;

import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.utils.Units;

public class Test {
	public static void main(String[] args) {
		final var pivot = new RigidTransform(new Translation(Units.inches2Meters(65), Units.inches2Meters(-14)), Rotation.degrees(-80));
		final var endPivot = pivot.transformBy(RigidTransform.fromTranslation(new Translation(Units.inches2Meters(36.0), 0)));
		System.out.println(endPivot.toString());
	}
}
