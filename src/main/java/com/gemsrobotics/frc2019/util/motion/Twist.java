package com.gemsrobotics.frc2019.util.motion;

import com.gemsrobotics.frc2019.util.Utils;

import java.text.DecimalFormat;

public class Twist {
	public static Twist identity() {
		return new Twist(0, 0, 0);
	}

	public final double dx, dy, dtheta;

	public Twist(final double x, final double y, final double theta) {
		dx = x;
		dy = y;
		dtheta = theta;
	}

	public Twist scaled(final double scalar) {
		return new Twist(dx * scalar, dy * scalar, dtheta * scalar);
	}

	private static final String REPR_STRING = "Twist[x: %s, y: %s, theta: %s]";

	@Override
	public String toString() {
		final DecimalFormat dcf = Utils.getDCF();
		return String.format(REPR_STRING, dcf.format(dx), dcf.format(dy), dcf.format(dtheta));
	}
}
