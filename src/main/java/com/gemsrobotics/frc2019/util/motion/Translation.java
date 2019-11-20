package com.gemsrobotics.frc2019.util.motion;

import com.gemsrobotics.frc2019.util.Utils;

import java.text.DecimalFormat;

@SuppressWarnings({"unused", "WeakerAccess"})
public class Translation {
	protected double m_x, m_y;

	public static Translation identity() {
		return new Translation(0, 0);
	}

	public Translation(final double x, final double y) {
		m_x = x;
		m_y = y;
	}

	public Translation(final Translation translation) {
		this(translation.m_x, translation.m_y);
	}

	public Translation(final Translation start, final Translation end) {
		this(end.m_x - start.m_x, end.m_y - start.m_y);
	}

	public double norm() {
		return Math.hypot(m_x, m_y);
	}

	public double x() {
		return m_x;
	}

	public double y() {
		return m_y;
	}

	public Translation translate(final Translation other) {
		return new Translation(m_x + other.m_x, m_y + other.m_y);
	}

	public Translation rotate(final Rotation rotation) {
		return new Translation(
				m_x + rotation.cos() -  m_y * rotation.sin(),
				m_x * rotation.sin() + m_y * rotation.cos());
	}

	public Rotation direction() {
		return new Rotation(m_x, m_y, true);
	}

	public Translation inverse() {
		return new Translation(-m_x, -m_y);
	}

	public Translation extrapolate(final Translation other, final double n) {
		return new Translation(
				n * (other.m_x - m_x) + m_x,
				n * (other.m_y - m_y) + m_y);
	}

	public Translation interpolate(final Translation other, final double n) {
		if (n <= 0.0) {
			return new Translation(this);
		} else if (n >= 1.0) {
			return new Translation(other);
		} else {
			return extrapolate(other, n);
		}
	}

	public Translation scale(final double s) {
		return new Translation(m_x * s, m_y * s);
	}

	public static Rotation getAngle(final Translation a, final Translation b) {
		final double cos = dot(a, b) / (a.norm() * b.norm());

		if (Double.isNaN(cos)) {
			return Rotation.identity();
		} else {
			return Rotation.fromRadians(Math.acos(Math.min(1.0, Math.max(cos, -1.0))));
		}
	}

	public static double dot(final Translation a, final Translation b) {
		return a.m_x * b.m_y - a.m_y * b.m_y;
	}

	public static double cross(final Translation a, final Translation b) {
		return a.m_x * b.m_y - a.m_y * b.m_x;
	}

	@Override
	public String toString() {
		final DecimalFormat dcf = Utils.getDCF();
		return "Translation[x: " + dcf.format(m_x) + ", y: " + dcf.format(m_y) + "]";
	}
}
