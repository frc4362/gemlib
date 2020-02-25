package com.gemsrobotics.frc2020;

import com.gemsrobotics.lib.data.InterpolatingTreeMap;
import com.gemsrobotics.lib.math.PolynomialRegression;

public class Constants {
	private Constants() { }

	public static final boolean USE_INNER_ADJUSTMENT = false;

	public static final double OUTER_TO_INNER_DISTANCE = 0.74295;

	public static final double WALL_SHOOTING_RPM = 0.0;
	public static final double[][] SHOOTER_RANGE_RPM = {
			{ 0.0, 0.0 },
			{ 1.0, 1.0 },
			{ 2.0, 0.0 }
	};
	public static final PolynomialRegression SHOOTER_RANGE_REGRESSION = PolynomialRegression.of(SHOOTER_RANGE_RPM, 2);

	public static final int[] DRIVE_PORTS = { 1, 2, 3, 4 };
	public static final int FEEDER_MASTER_PORT = 5;
	public static final int FEEDER_SLAVE_PORT = 6;
	public static final int SHOOTER_MASTER_PORT = 7;
	public static final int SHOOTER_SLAVE_PORT = 8;

	public static final int HOPPER_PORT = 12;
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
