package com.gemsrobotics.frc2022;

import com.gemsrobotics.lib.data.InterpolatingTreeMap;
import com.gemsrobotics.lib.math.interpolation.InterpolatingDouble;

import java.util.Optional;

public class Constants {
	private static final double[][] SHOOTER_RANGE_MPS = {
			{ 1.39, 8.75 },
			{ 1.67, 9.5 },
			{ 1.97, 10.25 },
			{ 2.24, 10.5 },
			{ 2.50, 11.0 },
			{ 2.80, 12.5 },
			{ 3.25, 14.0 }
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
			return 8.75;
		} else if (range > SHOOTER_RANGE_MPS[SHOOTER_RANGE_MPS.length - 1][0]) {
			return 14.0;
		} else {
			return SHOOTER_RANGE_RPM_LERPER_CLOSE.getInterpolated(new InterpolatingDouble(range)).value;
		}
	}
}
