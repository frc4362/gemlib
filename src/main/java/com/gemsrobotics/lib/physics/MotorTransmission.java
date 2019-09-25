package com.gemsrobotics.lib.physics;

import static java.lang.Math.max;
import static java.lang.Math.min;
import static com.gemsrobotics.lib.utils.MathUtils.kEpsilon;

/**
 * Model of a DC motor rotating a shaft.  All parameters refer to the output
 * (e.g. should already consider gearing and efficiency losses).
 * The motor is assumed to be symmetric forward/reverse.
 */
@SuppressWarnings("WeakerAccess")
public final class MotorTransmission {
    public static class Properties {
		double speedPerVolt, torquePerVolt, stictionVoltage;
	}

	// all units must be SI
	public final double speedPerVolt; // rad/s per V (no load)
	public final double torquePerVolt; // N m per V (stall)
	public final double stictionVoltage; // V

    public MotorTransmission(final double v, final double a, final double s) {
        this(new Properties() {
            {
                speedPerVolt = v;
                torquePerVolt = a;
                stictionVoltage = s;
            }
        });
    }

	public MotorTransmission(final Properties properties) {
		speedPerVolt = properties.speedPerVolt;
		torquePerVolt = properties.torquePerVolt;
		stictionVoltage = properties.stictionVoltage;
	}

	public double freeSpeedAtVoltage(final double voltage) {
		double effectiveVoltage = 0.0;

		if (voltage > kEpsilon) {
			// rolling forward, friction negative
			effectiveVoltage = max(0.0, voltage - stictionVoltage);
		} else if (voltage < -kEpsilon) {
			// rolling backward, friction positive
			effectiveVoltage = min(0.0, voltage + stictionVoltage);
		}

		return effectiveVoltage * speedPerVolt;
	}

	public double torqueForVoltage(final double velocity, final double voltage) {
		double effectiveVoltage = voltage;

		if (velocity > kEpsilon) {
			// rolling forward, friction negative
			effectiveVoltage -= stictionVoltage;
		} else if (velocity < -kEpsilon) {
			// rolling backward, friction positive
			effectiveVoltage += stictionVoltage;
		} else if (voltage > kEpsilon) {
			// static, with positive voltage
			effectiveVoltage = max(0.0, voltage - stictionVoltage);
		} else if (voltage < -kEpsilon) {
			// static, with negative voltage
			effectiveVoltage = min(0.0, voltage + stictionVoltage);
		} else {
			return 0.0;
		}

		return torquePerVolt * (effectiveVoltage - (velocity / speedPerVolt));
	}

	public double voltageFromTorque(final double velocity, final double torque) {
		double effectiveStiction;

		if (velocity > kEpsilon) {
			// rolling forward, friction negative
			effectiveStiction = stictionVoltage;
		} else if (velocity < -kEpsilon) {
			// rolling backward, friction positive
			effectiveStiction = -stictionVoltage;
		} else if (torque > kEpsilon) {
			// static, wants net torque forward
			effectiveStiction = stictionVoltage;
		} else if (torque < -kEpsilon) {
			// static, wants net torque backward
			effectiveStiction = -stictionVoltage;
		} else {
		    // system idle
			return 0.0;
		}

		return (torque / torquePerVolt) + (velocity / speedPerVolt) + effectiveStiction;
	}
}
