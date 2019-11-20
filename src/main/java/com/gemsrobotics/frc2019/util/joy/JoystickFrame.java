package com.gemsrobotics.frc2019.util.joy;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Packages all possible {@link Gemstick} axis values in one frame
 * with a few abstractions
 */
@SuppressWarnings({"unused", "WeakerAccess"})
public final class JoystickFrame {
	private final double m_x, m_y, m_z, m_magnitude, m_azimuth;
	private final long m_time;

	/**
	 * Provides the information about a {@link Gemstick} pose
	 * from just the base coordinates
	 */
	public JoystickFrame(final double x, final double y, final double z) {
		m_x = x;
		m_y = y;
		m_z = z;

		m_magnitude = Math.sqrt(Math.pow(m_x, 2) + Math.pow(m_y, 2));
		m_azimuth = (Math.toDegrees(Math.atan2(x, y)) + 360) % 360;

		m_time = System.currentTimeMillis();
	}

	/**
	 * @param stick The joystick from which you want to pull values
	 */
	public JoystickFrame(final Joystick stick) {
		this(stick.getRawAxis(0), stick.getRawAxis(1), stick.getRawAxis(2));
	}

	public double getX() {
		return m_x;
	}
	
	public double getY() {
		return m_y;
	}

	public double getZ() {
		return m_z;
	}

	public double getMagnitude() {
		return m_magnitude;
	}

	public double getAzimuth() {
		return m_azimuth;
	}
	
	public long getTime() {
		return m_time;
	}
}
