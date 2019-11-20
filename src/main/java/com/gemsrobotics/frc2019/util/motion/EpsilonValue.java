package com.gemsrobotics.frc2019.util.motion;

@SuppressWarnings("WeakerAccess")
public class EpsilonValue {
	// please note there is nothing here
	private EpsilonValue() { }

	public static final double Epsilon = 1e-9;

	public static boolean epsilonEquals(final double a, final double b) {
		return (a - Epsilon <= b) && (a + Epsilon >= b);
	}
}
