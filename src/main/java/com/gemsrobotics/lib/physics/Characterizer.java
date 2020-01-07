package com.gemsrobotics.lib.physics;

import com.gemsrobotics.lib.controls.MotorFeedforward;
import com.gemsrobotics.lib.math.PolynomialRegression;
import com.gemsrobotics.lib.telemetry.reporting.Reportable;
import com.gemsrobotics.lib.utils.MathUtils;

import java.util.List;
import java.util.Objects;

public final class Characterizer implements Reportable {
    public static class VelocityDataPoint {
        protected final double velocity, power;

        public VelocityDataPoint(final double velocity, final double power) {
            this.velocity = velocity;
            this.power = power;
        }
    }

    public static class AccelerationDataPoint {
        protected final double velocity, power, acceleration;

        public AccelerationDataPoint(final double velocity, final double power, final double acceleration) {
            this.velocity = velocity;
            this.power = power;
            this.acceleration = acceleration;
        }
    }

    public static MotorFeedforward.Constants generateCharacterization(final List<VelocityDataPoint> velocityData, final List<AccelerationDataPoint> accelerationData) {
        final var ret = doVelocityCharacterization(sanitizeVelocities(velocityData));
        return doAccelerationCharacterization(getAccelerationData(accelerationData, ret), ret);
    }

    private static MotorFeedforward.Constants doVelocityCharacterization(final double[][] points) {
        final MotorFeedforward.Constants constants = new MotorFeedforward.Constants();

        if (Objects.isNull(points)) {
            return constants;
        }

        final var p = PolynomialRegression.of(points, 1);

        constants.kStiction = p.beta(0);
        constants.kV = p.beta(1);

        return constants;
    }

    private static MotorFeedforward.Constants doAccelerationCharacterization(final double[][] points, final MotorFeedforward.Constants velocityChacterization) {
        if (Objects.isNull(points)) {
            return velocityChacterization;
        }

        final var p = PolynomialRegression.of(points, 1);

        velocityChacterization.kA = p.beta(1);

        return velocityChacterization;
    }

    private static double[][] getAccelerationData(final List<AccelerationDataPoint> input, final MotorFeedforward.Constants constants) {
        double[][] output = new double[input.size()][2];

        for (int i = 0; i < input.size(); ++i) {
            output[i][0] = input.get(i).acceleration;
            output[i][1] = input.get(i).power - constants.kV * input.get(i).velocity - constants.kStiction;
        }

        return output;
    }

    /**
     * removes data points with a velocity of zero to get a better line fit
     */
    private static double[][] sanitizeVelocities(final List<VelocityDataPoint> input) {
        double[][] output = null;
        int startTrim = 0;

        for (int i = 0; i < input.size(); ++i) {
            if (input.get(i).velocity > MathUtils.Epsilon) {
                if (Objects.isNull(output)) {
                    output = new double[input.size() - i][2];
                    startTrim = i;
                }

                output[i - startTrim][0] = input.get(i).velocity;
                output[i - startTrim][1] = input.get(i).power;
            }
        }

        return output;
    }
}
