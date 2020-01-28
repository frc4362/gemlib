package com.gemsrobotics.lib.controls;

import static java.lang.Math.signum;

public class MotorFeedforward {
    public static class Constants {
        public double kStiction; // voltage needed to break static friction
        public double kV; // v / rad/s
        public double kA; // v / rad/s^2
    }

    protected final double kS, kV, kA;

    public MotorFeedforward(final Constants constants) {
        this(constants.kStiction, constants.kV, constants.kA);
    }

    public MotorFeedforward(final double kS, final double kV, final double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }

    public double calculateVolts(final double velocityRadiansPerSecond, final double accelerationRadiansPerSecondPerSecond) {
        return kS * signum(velocityRadiansPerSecond)
               + kV * velocityRadiansPerSecond
               + kA * accelerationRadiansPerSecondPerSecond;
    }

    public double calculateVolts(final double velocityRadiansPerSecond) {
        return calculateVolts(velocityRadiansPerSecond, 0.0);
    }
}
