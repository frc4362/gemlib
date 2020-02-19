package com.gemsrobotics.lib.data;

import com.gemsrobotics.lib.math.interpolation.Interpolatable;
import com.gemsrobotics.lib.utils.MathUtils;

public class RollingAverage<T extends Interpolatable<T>> extends LimitedQueue<T> {
	public RollingAverage(final int capacity) {
		super(capacity);
	}

	public T getAverage() {
		return MathUtils.average(this);
	}
}
