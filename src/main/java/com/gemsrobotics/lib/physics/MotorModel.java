package com.gemsrobotics.lib.physics;

import static java.lang.Math.max;
import static java.lang.Math.min;
import static com.gemsrobotics.lib.utils.MathUtils.Epsilon;

/**
 * Model of a DC motor rotating a shaft.  All parameters refer to the output
 * (e.g. should already consider gearing and efficiency losses).
 * The motor is assumed to be symmetric forward/reverse.
 */
@SuppressWarnings("WeakerAccess")
public final class MotorModel {
    public static class Properties {
		public double speedRadiansPerSecondPerVolt, torquePerVolt, stictionVoltage;
	}

	// all units must be SI
	public final double speedRadiansPerSecondPerVolt; // rad/s per V (no load)
	public final double torquePerVolt; // N m per V (stall)
	public final double stictionVoltage; // V

	public MotorModel(final Properties properties) {
		speedRadiansPerSecondPerVolt = properties.speedRadiansPerSecondPerVolt;
		torquePerVolt = properties.torquePerVolt;
		stictionVoltage = properties.stictionVoltage;
	}

	public double freeSpeedAtVoltageRadiansPerSecond(final double voltage) {
		double effectiveVoltage = 0.0;

		if (voltage > Epsilon) {
			// rolling forward, friction negative
			effectiveVoltage = max(0.0, voltage - stictionVoltage);
		} else if (voltage < -Epsilon) {
			// rolling backward, friction positive
			effectiveVoltage = min(0.0, voltage + stictionVoltage);
		}

		return effectiveVoltage * speedRadiansPerSecondPerVolt;
	}

	public double torqueForVoltage(final double velocityRadiansPerSecond, final double voltage) {
		double effectiveVoltage = voltage;

		if (velocityRadiansPerSecond > Epsilon) {
			// rolling forward, friction negative
			effectiveVoltage -= stictionVoltage;
		} else if (velocityRadiansPerSecond < -Epsilon) {
			// rolling backward, friction positive
			effectiveVoltage += stictionVoltage;
		} else if (voltage > Epsilon) {
			// static, with positive voltage
			effectiveVoltage = max(0.0, voltage - stictionVoltage);
		} else if (voltage < -Epsilon) {
			// static, with negative voltage
			effectiveVoltage = min(0.0, voltage + stictionVoltage);
		} else {
			return 0.0;
		}

		return torquePerVolt * (effectiveVoltage - (velocityRadiansPerSecond / speedRadiansPerSecondPerVolt));
	}

	public double voltageFromTorque(final double velocityRadiansPerSecond, final double torque) {
		double effectiveStiction;

		if (velocityRadiansPerSecond > Epsilon) {
			// rolling forward, friction negative
			effectiveStiction = stictionVoltage;
		} else if (velocityRadiansPerSecond < -Epsilon) {
			// rolling backward, friction positive
			effectiveStiction = -stictionVoltage;
		} else if (torque > Epsilon) {
			// static, wants net torque forward
			effectiveStiction = stictionVoltage;
		} else if (torque < -Epsilon) {
			// static, wants net torque backward
			effectiveStiction = -stictionVoltage;
		} else {
		    // system idle
			return 0.0;
		}

		return effectiveStiction + (torque / torquePerVolt) + (velocityRadiansPerSecond / speedRadiansPerSecondPerVolt);
	}
}
