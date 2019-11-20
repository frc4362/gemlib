package com.gemsrobotics.frc2019.util.math;

@SuppressWarnings({"unused", "WeakerAccess"})
public class InterpolateDouble implements Interpolate<InterpolateDouble>,
  	InverseInterpolate<InterpolateDouble>, Comparable<InterpolateDouble>
{
	public Double value;

	public InterpolateDouble(final Double val) {
		value = val;
	}

	@Override
	public InterpolateDouble interpolate(final InterpolateDouble other, final double n) {
		return new InterpolateDouble((other.value - value) * n + value);
	}

	@Override
	public double inverseInterpolate(
			final InterpolateDouble upper,
			final InterpolateDouble query
	) {
		final double upperToLower = upper.value - value,
				queryToLower = query.value - value;

		if (upperToLower <= 0.0) {
			return 0.0;
		} else if (queryToLower <= 0.0) {
			return 0.0;
		} else {
			return queryToLower / upperToLower;
		}
	}

	@Override
	public int compareTo(final InterpolateDouble other) {
		if (other.value < value) {
			return 1;
		} else if (other.value > value) {
			return -1;
		} else {
			return 0;
		}
	}
}
