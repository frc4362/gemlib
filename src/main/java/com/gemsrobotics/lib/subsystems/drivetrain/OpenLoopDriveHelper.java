package com.gemsrobotics.lib.subsystems.drivetrain;

import static com.gemsrobotics.lib.utils.MathUtils.Tau;
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
        reset();
	}

    public void reset() {
        m_lastWheel = 0.0;
        m_quickStopAccumulator = 0.0;
        m_negInertiaAccumulator = 0.0;
    }

	public WheelState drive(
			double throttle,
			double wheel,
			final boolean isQuickTurn,
			final boolean isHighGear
	) {
		double negInertia = wheel - m_lastWheel;
		m_lastWheel = wheel;

		if (m_cfg.useSineAttack) {
			final double zNonLinearity;

            // Apply a sin function s scaled to make it feel better.
			if (isHighGear) {
				zNonLinearity = m_cfg.zNonLinearityHighGear;

				final double denominator = sin(Tau / 4.0 * zNonLinearity);
				wheel = sin(Tau / 4.0 * zNonLinearity * wheel) / denominator;
				wheel = sin(Tau / 4.0 * zNonLinearity * wheel) / denominator;
			} else {
				zNonLinearity = m_cfg.zNonLinearityLowGear;

				final double denominator = sin(Tau / 4.0 * zNonLinearity);
				wheel = sin(Tau / 4.0 * zNonLinearity * wheel) / denominator;
				wheel = sin(Tau / 4.0 * zNonLinearity * wheel) / denominator;
				wheel = sin(Tau / 4.0 * zNonLinearity * wheel) / denominator;
			}
		}

		final double negativeInertiaScalar, sensitivity;

		if (isHighGear) {
			negativeInertiaScalar = m_cfg.negativeInertiaScalarHigh;
			sensitivity = m_cfg.sensitivityHighGear;
		} else {
			sensitivity = m_cfg.sensitivityLowGear;

            // If we are moving away from 0.0, aka, trying to turn more.
			if (wheel * negInertia > 0) {
				negativeInertiaScalar = m_cfg.negativeInertiaTurnScalarLow;
			} else if (abs(wheel) > m_cfg.negativeInertiaThresholdLow) {
                negativeInertiaScalar = m_cfg.negativeInertiaFarScalarLow;
            } else {
                negativeInertiaScalar = m_cfg.negativeInertiaCloseScalarLow;
            }
		}

//        System.out.println("neg inertia scalar: " + negativeInertiaScalar);

        final double negativeInteriaPower = negInertia * negativeInertiaScalar;
//        System.out.println("neg inertia power: " + negativeInteriaPower);
        m_negInertiaAccumulator += negativeInteriaPower;

		wheel += negativeInteriaPower;

		if (m_negInertiaAccumulator > 1.0) {
			m_negInertiaAccumulator -= 1.0;
		} else if (m_negInertiaAccumulator < -1.0) {
			m_negInertiaAccumulator += 1.0;
		} else {
			m_negInertiaAccumulator = 0.0;
		}

		final ChassisState powers = new ChassisState();
		powers.linearMeters = throttle;

		final double overPower;

		if (isQuickTurn) {
			if (abs(powers.linearMeters) < m_cfg.quickStopDeadband) {
				m_quickStopAccumulator = (1.0 - m_cfg.quickStopWeight) * m_quickStopAccumulator;
				m_quickStopAccumulator += m_cfg.quickStopWeight * limit(wheel, 1.0) * m_cfg.quickStopScalar;
			}

			overPower = 1.0;
			powers.angularRadians = wheel;
		} else  {
			overPower = 0.0;
			powers.angularRadians = abs(throttle) * (wheel * signum(throttle)) * sensitivity - m_quickStopAccumulator;

			if (m_quickStopAccumulator > 1.0) {
				m_quickStopAccumulator -= 1.0;
			} else if (m_quickStopAccumulator < -1.0) {
				m_quickStopAccumulator += 1.0;
			} else {
				m_quickStopAccumulator = 0;
			}
		}

        final WheelState output = new WheelState(powers.linearMeters - powers.angularRadians, powers.linearMeters + powers.angularRadians);

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
}
