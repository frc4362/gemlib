package com.gemsrobotics.lib.math.interpolation;

import com.gemsrobotics.lib.data.InterpolatingTreeMap;

/**
 * A Long that can be interpolated using the InterpolatingTreeMap.
 * 
 * @see InterpolatingTreeMap
 */
public class InterpolatingLong implements
        Interpolable<InterpolatingLong>,
        InverseInterpolable<InterpolatingLong>,
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
        final var upper_to_lower = upper.value - value;
        if (upper_to_lower <= 0) {
            return 0;
        }

        final var query_to_lower = query.value - value;
        if (query_to_lower <= 0) {
            return 0;
        }

        return query_to_lower / (double) upper_to_lower;
    }

    @Override
    public int compareTo(InterpolatingLong other) {
        if (other.value < value) {
            return 1;
        } else if (other.value > value) {
            return -1;
        } else {
            return 0;
        }
    }
}
