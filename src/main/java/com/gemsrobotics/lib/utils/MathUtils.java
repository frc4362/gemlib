package com.gemsrobotics.lib.utils;

import com.gemsrobotics.lib.math.interpolation.Interpolatable;

import java.util.Collection;
import java.util.List;
import java.util.stream.DoubleStream;

import static java.lang.Math.*;

@SuppressWarnings("unused")
public class MathUtils {
    public static final double
            Tau = 2 * PI,
            Epsilon = 1e-5;

    // See: https://floating-point-gui.de/errors/comparison/
    public static boolean epsilonEquals(final double a, final double b, final double epsilon) {
        final double
                absA = abs(a),
                absB = abs(b),
                diff = abs(a - b);

        // shortcut, handles infinities
        if (a == b) {
            return true;
        } else if (a == 0 || b == 0 || (absA + absB < Double.MIN_NORMAL)) {
            // a or b is zero or both are extremely close to it
            // relative error is less meaningful here
            return diff < epsilon;
        } else {
            // use relative error
            return diff / min((absA + absB), Double.MAX_VALUE) < epsilon;
        }
    }

    public static boolean epsilonEquals(final double a, final double b) {
        return epsilonEquals(a, b, Epsilon);
    }

    public static boolean allCloseTo(final Collection<Double> nums, final double value, final double epsilon) {
        return nums.stream().allMatch(n -> epsilonEquals(n, value, epsilon));
    }

    public static boolean allCloseTo(final Collection<Double> nums, final double value) {
        return allCloseTo(nums, value, Epsilon);
    }

    public static double coerce(final double bot, double v, final double top) {
        return min(max(v, bot), top);
    }

    public static boolean inRange(final double min, double v, final double max) {
        return (v <= max) && (v >= min);
    }

	public static double limit(final double v, final double magnitude) {
		return coerce(-magnitude, v, magnitude);
	}

	public static double lerp(final double a, final double b, double x) {
		x = coerce(0.0, x, 1.0);

        // this slightly algebraically inefficient method maintains precision
        // when a and b are of significantly different magnitudes
        return (a * (1.0 - x)) + (b * x);
	}

    // this is a historical function
    // just google "fast inverse square root" if you don't know
    public static double inverseSqrt(double n) {
        final double half = 0.5 * n;
        long raw = Double.doubleToLongBits(n);
        raw = 0x5fe6ec85e7de30daL - (raw >> 1);
        n = Double.longBitsToDouble(raw);

        for (int i = 0; i < 4; i++){
            n *= (1.5 - half * n * n);
        }

        return n;
    }

    // See: https://stackoverflow.com/questions/35666078/fast-integer-powers-in-java
    public static double fastLongPow(long a, long b) {
        long result = 1;

        while (b > 0) {
            if ((b & 1) == 0) {
                a *= a;
                b >>>= 1;
            } else {
                result *= a;
                b--;
            }
        }

        return result;
    }

    public static double sinc(final double x, final double epsilon) {
	    if (epsilonEquals(x, 0.0, epsilon)) {
	        return 1.0 - 1.0 / 6.0 * x * x;
        } else {
	        return sin(x) / x;
        }
    }

    public static double sinc(final double x) {
        return sinc(x, Epsilon);
    }

    // for averaging objects, for instance Translations of vision targets
	public static <T extends Interpolatable<T>> T average(final List<T> ns) {
        T ret = ns.get(0);

        // this could be simplified by starting the harmonic sequence from 1
        // but there's no way to get the identity element of the interpolatable
        for (int i = 1; i < ns.size(); i++) {
            // The terms of the sequence are interpolated
            // with harmonically decreasing proportion
            ret = ret.interpolate(ns.get(i), 1 / (i + 1.0));
        }

        return ret;
    }

    public static final class Bounds {
        public double min, max;

        public Bounds(final double min, final double max) {
            this.min = min;
            this.max = max;
        }

        public double coerce(final double val) {
            return MathUtils.coerce(min, val, max);
        }

        public boolean isValid(final double val) {
            return inRange(min, val, max);
        }

        @Override
        public String toString() {
            return "[" + FastDoubleToString.format(min) + ", " + FastDoubleToString.format(max) + "]";
        }
    }
}
