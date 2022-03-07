package com.gemsrobotics.frc2022;

import com.gemsrobotics.lib.data.InterpolatingTreeMap;
import com.gemsrobotics.lib.math.interpolation.InterpolatingDouble;

public class Constants {
	private static final double[][] SHOOTER_RANGE_MPS = {
			{ 1.42, 9.00 },
			{ 1.88, 9.5 },
			{ 2.51, 10.35 }
	};

	private static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> SHOOTER_RANGE_RPM_LERPER_CLOSE;
	static {
		SHOOTER_RANGE_RPM_LERPER_CLOSE = new InterpolatingTreeMap<>(SHOOTER_RANGE_MPS.length);

		for (final var tuning : SHOOTER_RANGE_MPS) {
			SHOOTER_RANGE_RPM_LERPER_CLOSE.put(new InterpolatingDouble(tuning[0]), new InterpolatingDouble(tuning[1]));
		}
	}

	public static double getRPM(final double range) {
		if (range < SHOOTER_RANGE_MPS[0][0]) {
			return 9.00;
		} else if (range > SHOOTER_RANGE_MPS[SHOOTER_RANGE_MPS.length - 1][0]) {
			return 10.35;
		} else {
			return SHOOTER_RANGE_RPM_LERPER_CLOSE.getInterpolated(new InterpolatingDouble(range)).value;
		}
	}
}
