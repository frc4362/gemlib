package com.gemsrobotics.lib.controls;

import com.gemsrobotics.lib.utils.MathUtils.Bounds;

import static com.gemsrobotics.lib.utils.MathUtils.kEpsilon;
import static java.lang.Math.abs;

public class PIDFController {
    public static class Gains {
        public double kP, kI, kD, kFF, allowableError;

        public Gains(final double p, final double i, final double d, final double ff) {
            kP = p;
            kI = i;
            kD = d;
            kFF = ff;
            allowableError = 0.0;
        }

        public Gains(final double p, final double i, final double d, final double ff, final int error) {
            kP = p;
            kI = i;
            kD = d;
            kFF = ff;
            allowableError = error;
        }
    }

    private Gains m_gains;
    private double m_reference;

    private boolean m_continuous;
    private double m_inputRange;
    private Bounds m_outputRange;

    private double m_lastError;
    private double m_integralAccum;
    private double m_integralRange;
    private double m_allowedError;

    public PIDFController(final Gains constants) {
        m_gains = constants;
        m_continuous = true;
        m_inputRange = Double.POSITIVE_INFINITY;
        m_outputRange = new Bounds(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

        m_lastError = 0.0;
        m_integralAccum = 0.0;
        m_integralRange = Double.POSITIVE_INFINITY;

        m_allowedError = constants.allowableError;
    }

    public PIDFController(final double p, final double i, final double d, final double f) {
        this(new Gains(p, i, d, f));
    }

    public PIDFController(final double p, final double i, final double d) {
        this(p, i, d, 0);
    }

    public double update(final double input, double dt) {
        if (dt < kEpsilon) {
            dt = kEpsilon;
        }

        double error = m_reference - input;

        if (m_continuous) {
            error %= m_inputRange;

            if (abs(error) > m_inputRange / 2.0) {
                if (error > 0.0) {
                    error -= m_inputRange;
                } else {
                    error += m_inputRange;
                }
            }
        }

        double integral = 0.0;

        if (abs(error) > m_integralRange / 2.0) {
            integral = m_integralAccum + error * dt;
        }

        m_integralAccum = integral;

        double derivative = 0.0;

        if (Double.isFinite(m_lastError)) {
            derivative = (error - m_lastError) / dt;
        }

        m_lastError = error;

        // only apply deadband to proportional response, maintains derivative
        final var proportional = abs(error) < m_allowedError ? 0.0 : error;

        return m_outputRange.constrain(
                m_gains.kP * proportional
                    + m_gains.kI * integral
                    + m_gains.kD * derivative
                    + m_gains.kFF * m_reference);
    }

    public void reset() {
        m_lastError = Double.NaN;
        m_integralAccum = 0.0;
    }

    public double getReference() {
        return m_reference;
    }

    public void setReference(final double reference) {
        m_reference = reference;
    }

    public void setContinuous(final boolean continuous) {
        m_continuous = continuous;
    }

    public void setInputRange(final double minInput, final double maxInput) {
        m_inputRange = maxInput - minInput;
    }

    public void setIntegralRange(final double integralRange) {
        m_integralRange = integralRange;
    }

    /**
     * Sets the output range for the controller. Outputs will be clamped between these two values.
     *
     * @param min the minimum allowable output value
     * @param max the maximum allowable output value
     */
    public void setOutputRange(final double min, final double max) {
        if (max < min) {
            throw new IllegalArgumentException("Minimum output cannot be greater than maximum output");
        }

        m_outputRange = new Bounds(min, max);
    }

    public void setDeadband(final double deadband) {
        m_allowedError = abs(deadband);
    }

    public boolean isOnTarget(final double tolerance) {
        return !Double.isNaN(m_lastError) && abs(m_lastError) < tolerance;
    }
}
