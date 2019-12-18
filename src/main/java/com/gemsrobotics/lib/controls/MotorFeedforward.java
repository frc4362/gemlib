package com.gemsrobotics.lib.controls;

import static java.lang.Math.signum;

public class MotorFeedforward {
    public static class Constants {
        public double kStiction; // voltage needed to break static friction
        public double kV; // v / rad/s
        public double kA; // v / rad/s^2
    }

    protected final Constants m_constants;

    public MotorFeedforward(final Constants constants) {
        m_constants = constants;
    }

    public double calculateVolts(final double velocityRadiansPerSecond, final double accelerationRadiansPerSecondPerSecond) {
        return m_constants.kStiction * signum(velocityRadiansPerSecond)
               + m_constants.kV * velocityRadiansPerSecond
               + m_constants.kA * accelerationRadiansPerSecondPerSecond;
    }

    public double calculateVolts(final double velocityRadiansPerSecond) {
        return calculateVolts(velocityRadiansPerSecond, 0.0);
    }
}
