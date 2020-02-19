package com.gemsrobotics.frc2020;

import com.gemsrobotics.lib.data.InterpolatingTreeMap;
import com.gemsrobotics.lib.math.PolynomialRegression;

public class Constants {
	private Constants() { }

	public static final int HOPPER_MOTOR_PORT = 7;
	public static final int HOPPER_COLOR_SENSOR_PORT = 1;
	public static final int TURRET_PORT = 39;

	public static final int CHANNEL_LEFT_PORT = 0;
	public static final int CHANNEL_CENTER_PORT = 0;
	public static final int CHANNEL_RIGHT_PORT = 0;

	public static final int HOOD_SOLENOID_PORT = 0;

	public static final double[][] SHOOTER_RANGE_RPM = {
			{ 0.0, 0.0 },
			{ 1.0, 1.0 },
			{ 2.0, 0.0 }
	};

	public static final PolynomialRegression SHOOTER_RANGE_REGRESSION = PolynomialRegression.of(SHOOTER_RANGE_RPM, 2);
}
