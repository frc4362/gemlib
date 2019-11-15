package com.gemsrobotics.lib.subsystems.drivetrain;

import static com.gemsrobotics.lib.utils.MathUtils.limit;
import com.google.gson.annotations.SerializedName;

import static java.lang.Math.*;

@SuppressWarnings({"unused", "WeakerAccess"})
public class OpenLoopDriveHelper {
	public static class Config {
		boolean useSineAttack;

		double zNonLinearityHighGear, zNonLinearityLowGear;
		double sensitivityHighGear, sensitivityLowGear;

		double negativeInertiaScalarHigh;

		double negativeInertiaThresholdLow;
		double negativeInertiaTurnScalarLow;
		double negativeInertiaCloseScalarLow;
		double negativeInertiaFarScalarLow;

		double quickStopDeadband;
		double quickStopWeight;
		double quickStopScalar;
	}

	protected final transient OpenLoopDriveHelper.Config m_cfg;

	@SerializedName("lastWheel")
	protected double m_lastWheel;
	@SerializedName("quickStopAccumulator")
	protected double m_quickStopAccumulator;
	@SerializedName("negativeInertia")
	protected double m_negInertiaAccumulator;

	public OpenLoopDriveHelper(final OpenLoopDriveHelper.Config config) {
		m_cfg = config;

		m_lastWheel = 0.0;
		m_quickStopAccumulator = 0.0;
		m_negInertiaAccumulator = 0.0;
	}

	public WheelState drive(
			double xPower,
			double zRotation,
			final boolean isQuickTurn,
			final boolean isHighGear
	) {
		double negInertia = zRotation - m_lastWheel;
		m_lastWheel = zRotation;

		if (m_cfg.useSineAttack) {
			final double zNonLinearity;

			if (isHighGear) {
				zNonLinearity = m_cfg.zNonLinearityHighGear;

				final double denominator = sin(PI / 2.0 * zNonLinearity);
				// Apply a sin function s scaled to make it feel better.
				zRotation = sin(PI / 2.0 * zNonLinearity * zRotation) / denominator;
				zRotation = sin(PI / 2.0 * zNonLinearity * zRotation) / denominator;
			} else {
				zNonLinearity = m_cfg.zNonLinearityLowGear;

				final double denominator = sin(PI / 2.0 * zNonLinearity);
				// Apply a sin function s scaled to make it feel better.
				zRotation = sin(PI / 2.0 * zNonLinearity * zRotation) / denominator;
				zRotation = sin(PI / 2.0 * zNonLinearity * zRotation) / denominator;
				zRotation = sin(PI / 2.0 * zNonLinearity * zRotation) / denominator;
			}
		}

		final double negativeInertiaScalar, sensitivity;

		if (isHighGear) {
			negativeInertiaScalar = m_cfg.negativeInertiaScalarHigh;
			sensitivity = m_cfg.sensitivityHighGear;
		} else {
			sensitivity = m_cfg.sensitivityLowGear;

			if (zRotation * negInertia > 0) {
				// If we are moving away from 0.0, aka, trying to turn more.
				negativeInertiaScalar = m_cfg.negativeInertiaTurnScalarLow;
			} else {
				if (abs(zRotation) > m_cfg.negativeInertiaThresholdLow) {
					negativeInertiaScalar = m_cfg.negativeInertiaFarScalarLow;
				} else {
					negativeInertiaScalar = m_cfg.negativeInertiaCloseScalarLow;
				}
			}
		}

		final double negInertiaPower = negInertia * negativeInertiaScalar;
		m_negInertiaAccumulator += negInertiaPower;

		xPower += negInertiaPower;

		if (m_negInertiaAccumulator > 1.0) {
			m_negInertiaAccumulator -= 1.0;
		} else if (m_negInertiaAccumulator < -1.0) {
			m_negInertiaAccumulator += 1.0;
		} else {
			m_negInertiaAccumulator = 0.0;
		}

		final ChassisState powers = new ChassisState();
		powers.linearMeters = xPower;

		final double overPower;

		if (isQuickTurn) {
			if (abs(powers.linearMeters) < m_cfg.quickStopDeadband) {
				m_quickStopAccumulator = (1.0 - m_cfg.quickStopWeight) * m_quickStopAccumulator;
				m_quickStopAccumulator += m_cfg.quickStopWeight * limit(zRotation, 1.0) * m_cfg.quickStopScalar;
			}

			overPower = 1.0;
			powers.angularRadians = zRotation;
		} else  {
			overPower = 0.0;
			powers.angularRadians = abs(xPower) * zRotation * sensitivity - m_quickStopAccumulator;

			if (m_quickStopAccumulator > 1.0) {
				m_quickStopAccumulator -= 1.0;
			} else if (m_quickStopAccumulator < -1.0) {
				m_quickStopAccumulator += 1.0;
			} else {
				m_quickStopAccumulator = 0;
			}
		}

		final WheelState output = new WheelState(powers.linearMeters + powers.angularRadians, powers.linearMeters - powers.angularRadians);

		if (output.left > 1.0) {
			output.right -= overPower * (output.left - 1.0);
			output.left = 1.0;
		} else if (output.right > 1.0) {
			output.left -= overPower * (output.right - 1.0);
			output.right = 1.0;
		} else if (output.left < -1.0) {
			output.right += overPower * (-output.left - 1.0);
			output.left = -1.0;
		} else if (output.right < -1.0) {
			output.left += overPower * (-output.right - 1.0);
			output.right = -1.0;
		}

		return output;
	}

	public void reset() {
	    m_lastWheel = 0.0;
	    m_quickStopAccumulator = 0.0;
	    m_negInertiaAccumulator = 0.0;
    }
}
