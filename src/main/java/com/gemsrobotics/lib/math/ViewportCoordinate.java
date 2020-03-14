package com.gemsrobotics.lib.math;

public final class ViewportCoordinate {
	protected static final double UNIT_DISTANCE = 1.0;

	// y
	protected final double m_y, m_z, m_skew;

	public ViewportCoordinate(final double y, final double z, final double skew) {
		m_y = y;
		m_z = z;
		m_skew = skew;
	}

	public double getX() {
		return UNIT_DISTANCE;
	}

	public double getY() {
		return m_y;
	}

	public double getZ() {
		return m_z;
	}

	public double getSkew() {
		return m_skew;
	}
}
