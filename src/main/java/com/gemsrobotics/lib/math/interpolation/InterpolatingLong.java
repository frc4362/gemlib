package com.gemsrobotics.lib.math.interpolation;

import com.gemsrobotics.lib.data.InterpolatingTreeMap;

/**
 * A Long that can be interpolated using the InterpolatingTreeMap.
 * 
 * @see InterpolatingTreeMap
 */
public class InterpolatingLong implements
        Interpolatable<InterpolatingLong>,
        InverseInterpolatable<InterpolatingLong>,
        Comparable<InterpolatingLong>
{
    public Long value;

    public InterpolatingLong(final Long val) {
        value = val;
    }

    @Override
    public InterpolatingLong interpolate(final InterpolatingLong other, final double x) {
        Long dydx = other.value - value;
        Double searchY = dydx * x + value;
        return new InterpolatingLong(searchY.longValue());
    }

    @Override
    public double inverseInterpolate(final InterpolatingLong upper, final InterpolatingLong query) {
        final var upperToLower = upper.value - value;
        if (upperToLower <= 0.0) {
            return 0.0;
        }

        final var queryToLower = query.value - value;
        if (queryToLower <= 0.0) {
            return 0.0;
        }

        return queryToLower / (double) upperToLower;
    }

    @Override
    public int compareTo(final InterpolatingLong other) {
        if (other.value < value) {
            return 1;
        } else if (other.value > value) {
            return -1;
        } else {
            return 0;
        }
    }
}
