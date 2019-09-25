package com.gemsrobotics.lib.utils;

import java.util.Collection;

import static java.lang.Math.abs;
import static java.lang.Math.min;

@SuppressWarnings("unused")
public class MathUtils {
	public static final double kEpsilon = 1e-6;

	private MathUtils() {
	}

	public static double limit(double v, double maxMagnitude) {
		return constrain(v, -maxMagnitude, maxMagnitude);
	}

	public static double lerp(final double a, final double b, double x) {
		x = constrain(0.0, x, 1.0);

        // this slightly algebraically inefficient method maintains precision
        // when a and b are of significantly different magnitudes
        return (a * (1.0 - x)) + (b * x);
	}

	// see: https://floating-point-gui.de/errors/comparison/
	public static boolean epsilonEquals(final double a, final double b, final double epsilon) {
		final double absA = abs(a);
		final double absB = abs(b);
		final double diff = abs(a - b);

		if (a == b) { // shortcut, handles infinities
			return true;
		} else if (a == 0 || b == 0 || (absA + absB < Double.MIN_NORMAL)) {
			// a or b is zero or both are extremely close to it
			// relative error is less meaningful here
			return diff < epsilon;
		} else { // use relative error
			return diff / min((absA + absB), Double.MAX_VALUE) < epsilon;
		}
	}

	public static boolean epsilonEquals(final double a, final double b) {
		return epsilonEquals(a, b, kEpsilon);
	}

	public static boolean allCloseTo(final Collection<Double> nums, final double value, final double epsilon) {
		return nums.stream().allMatch(n -> epsilonEquals(n, value, epsilon));
	}

	public static double constrain(final double bot, double v, final double top) {
        if (v < bot) {
            return bot;
        } else if (v > top) {
            return top;
        } else {
            return v;
        }
	}

    public static class Bounds {
        public double min;
        public double max;

        public Bounds(final double min, final double max) {
            this.min = min;
            this.max = max;
        }

        public double constrain(double val) {
            return MathUtils.constrain(min, val, max);
        }

        @Override
        public String toString() {
            return "(" + FastDoubleToString.format(min) + ", " + FastDoubleToString.format(max) + ")";
        }
    }
}
