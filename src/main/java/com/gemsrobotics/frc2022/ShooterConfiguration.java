package com.gemsrobotics.frc2022;

import com.gemsrobotics.frc2022.subsystems.Hood;
import com.gemsrobotics.lib.math.interpolation.Interpolatable;
import com.gemsrobotics.lib.math.interpolation.InterpolatingDouble;
import com.gemsrobotics.lib.math.se2.Rotation;

public class ShooterConfiguration implements Interpolatable<ShooterConfiguration> {
	public static final ShooterConfiguration NULL_CONFIGURATION = new ShooterConfiguration(Hood.MIN_ANGLE, new InterpolatingDouble(0.0));

	private final Rotation m_hoodAngle;
	private final InterpolatingDouble m_wheelSpeedMetersPerSecond;

	public ShooterConfiguration(final Rotation hoodAngle, final InterpolatingDouble wheelSpeedMetersPerSecond) {
		m_hoodAngle = hoodAngle;
		m_wheelSpeedMetersPerSecond = wheelSpeedMetersPerSecond;
	}

	@Override
	public ShooterConfiguration interpolate(final ShooterConfiguration other, final double x) {
		return new ShooterConfiguration(
				m_hoodAngle.interpolate(other.m_hoodAngle, x),
				m_wheelSpeedMetersPerSecond.interpolate(other.m_wheelSpeedMetersPerSecond, x));
	}

	public Rotation getHoodAngle() {
		return m_hoodAngle;
	}

	public double getWheelSpeedMetersPerSecond() {
		return m_wheelSpeedMetersPerSecond.value;
	}

	@Override
	public boolean equals(final Object obj) {
		if (!(obj instanceof ShooterConfiguration)) {
			return false;
		}

		final var other = (ShooterConfiguration) obj;

		return m_hoodAngle.isParallel(other.m_hoodAngle)
			   && m_wheelSpeedMetersPerSecond.compareTo(other.m_wheelSpeedMetersPerSecond) == 0;
	}
}
