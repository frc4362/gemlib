package com.gemsrobotics.frc2020;

import com.gemsrobotics.lib.data.InterpolatingTreeMap;
import com.gemsrobotics.lib.math.PolynomialRegression;
import com.gemsrobotics.lib.math.interpolation.InterpolatingDouble;
import com.gemsrobotics.lib.math.interpolation.InverseInterpolatable;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.utils.Units;

public class Constants {
	private Constants() { }

	public static final double FIELD_LENGTH_METERS = 16.4846;

	public static final boolean USE_UNCOUNTED_SHOOTING = true;
	public static final boolean USE_INNER_ADJUSTMENT = false;
	public static final Translation OUTER_TO_INNER = new Translation(0.74295, 0.0);

	public static final double MAX_SHOT_RANGE_METERS = Units.feet2Meters(40.0);
	public static final double CLOSE_SHOT_RANGE_METERS = Units.feet2Meters(5.0);

	public static final double WALL_SHOOTING_RPM = 0.0;
	public static final double[][] SHOOTER_RANGE_RPM_FAR = {
			{ 3.1, 4700.0 },
			{ 4.76, 4400.0 },
			{ 6.31, 4900.0 },
			{ 7.184, 5050.0 },
			{ 8.732, 5200.0 }
	};
	public static final PolynomialRegression SHOOTER_RANGE_REGRESSION_FAR = PolynomialRegression.of(SHOOTER_RANGE_RPM_FAR, 2);

	public static final double[][] SHOOTER_RANGE_RPM_CLOSE = {
			{ 0.0, 6000.0 },
			{ 1.79, 6000.0 },
			{ 2.65, 6000.0 },
			{ 3.09, 4700.0 },
	};
	public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> SHOOTER_RANGE_RPM_LERPER_CLOSE;
	static {
		SHOOTER_RANGE_RPM_LERPER_CLOSE = new InterpolatingTreeMap<>(3);

		for (final var tuning : SHOOTER_RANGE_RPM_CLOSE) {
			SHOOTER_RANGE_RPM_LERPER_CLOSE.put(new InterpolatingDouble(tuning[0]), new InterpolatingDouble(tuning[1]));
		}
	}

	public static double getRPM(final double range) {
		if (range < 3.1) {
			return SHOOTER_RANGE_RPM_LERPER_CLOSE.getInterpolated(new InterpolatingDouble(range)).value;
		} else {
			return SHOOTER_RANGE_REGRESSION_FAR.predict(range);
		}
	}

	public static final int CANIFIER_PORT = 60;

	public static final int[] DRIVE_PORTS = { 1, 2, 3, 4 };
	public static final int FEEDER_MASTER_PORT = 7;
	public static final int FEEDER_SLAVE_PORT = 8;
	public static final int SHOOTER_MASTER_PORT = 5;
	public static final int SHOOTER_SLAVE_PORT = 6;

	public static final int HOPPER_PORT = 40;
	public static final int TURRET_PORT = 13;
	public static final int CONTROL_PANEL_MOTOR_PORT = 14;

	public static final int CHANNEL_RIGHT_PORT = 30;
	public static final int CHANNEL_CENTER_PORT = 31;
	public static final int CHANNEL_LEFT_PORT = 32;

	public static final int HOOD_SOLENOID_PORT = 0;
	public static final int INTAKE_SOLENOID_PORT = 1;
	public static final int KICKER_SOLENOID_PORT = 2;
	public static final int[] PTO_SOLENOID_PORT = { 3, 4 };
}
