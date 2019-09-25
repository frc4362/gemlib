package com.gemsrobotics.lib.math.interpolation;

import com.gemsrobotics.lib.data.InterpolatingTreeMap;

/**
 * A Double that can be interpolated.
 * @see InterpolatingTreeMap
 */
@SuppressWarnings({"unused", "WeakerAccess"})
public class InterpolatingDouble implements
        Interpolable<InterpolatingDouble>,
        InverseInterpolable<InterpolatingDouble>,
        Comparable<InterpolatingDouble>
{
    public Double value;

    public InterpolatingDouble(Double val) {
        value = val;
    }

    @Override
    public InterpolatingDouble interpolate(final InterpolatingDouble other, final double x) {
        final var dydx = other.value - value;
        final var searchY = dydx * x + value;
        return new InterpolatingDouble(searchY);
    }

    @Override
    public double inverseInterpolate(final InterpolatingDouble upper, final InterpolatingDouble query) {
        final var upper_to_lower = upper.value - value;
        if (upper_to_lower <= 0) {
            return 0;
        }

        final var  query_to_lower = query.value - value;
        if (query_to_lower <= 0) {
            return 0;
        }

        return query_to_lower / upper_to_lower;
    }

    @Override
    public int compareTo(final InterpolatingDouble other) {
        if (other.value < value) {
            return 1;
        } else if (other.value > value) {
            return -1;
        } else {
            return 0;
        }
    }
}
