package com.gemsrobotics.frc2022;

import com.gemsrobotics.lib.data.InterpolatingTreeMap;
import com.gemsrobotics.lib.math.interpolation.InterpolatingDouble;

import java.util.Optional;

public class Constants {
	public static final boolean DO_SHOOTER_LOGGING = true;
	public static final boolean DO_EARLY_FLYWHEEL = true;
	public static final boolean DO_CARGO_REJECT = false;

//	private static final double[][] SHOOTER_RANGE_MPS = {
//			{ 1.39, 8.75 + 0.45 },
//			{ 1.67, 9.5 + 0.35 },
//			{ 1.97, 10.25 + 0.3 },
//			{ 2.24, 10.5 + 0.1},
//			{ 2.50, 11.0 },
//			{ 2.80, 12.5 },
//			{ 3.25, 12.5 }
//	};

	private static final double[][] SHOOTER_RANGE_MPS = {
			{ 1.25, 9.15 },
			{ 1.4, 9.25 },
			{ 1.55, 9.6 },
			{ 1.7, 9.8 },
			{ 2.00, 10.0 },
			{ 2.17, 10.7 },
			{ 2.45, 11.0 },
			{ 2.63, 12.0 },
			{ 3.1, 13.9 },
			{ 3.27, 14.2 }
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
			return SHOOTER_RANGE_MPS[0][1];
		} else if (range > SHOOTER_RANGE_MPS[SHOOTER_RANGE_MPS.length - 1][0]) {
			return SHOOTER_RANGE_MPS[SHOOTER_RANGE_MPS.length - 1][1];
		} else {
			return SHOOTER_RANGE_RPM_LERPER_CLOSE.getInterpolated(new InterpolatingDouble(range)).value;
		}
	}
}
