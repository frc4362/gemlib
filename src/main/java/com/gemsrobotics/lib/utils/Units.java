package com.gemsrobotics.lib.utils;

import static com.gemsrobotics.lib.utils.MathUtils.Tau;

public class Units {
    private Units() {}

    public static double degrees2Rads(final double degrees) {
        return degrees * 0.0174533;
    }

    public static double rads2Degrees(final double radians) {
        return radians * 57.2958;
    }

	public static double rpm2RadsPerSecond(final double rpm) {
		return rpm * Tau / 60.0;
	}

	public static double radsPerSec2Rpm(final double radiansPerSecond) {
		return radiansPerSecond * 60.0 / Tau;
	}

	public static double inches2Meters(final double inches) {
		return inches * 0.0254;
	}

	public static double meters2Inches(final double meters) {
		return meters / 0.0254;
	}

	public static double feet2Meters(final double feet) {
        return inches2Meters(12 * feet);
    }

    public static double meters2Feet(final double meters) {
        return meters2Inches(meters) / 12.0;
    }
}
