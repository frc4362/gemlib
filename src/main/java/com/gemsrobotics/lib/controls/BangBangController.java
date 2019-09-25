package com.gemsrobotics.lib.controls;

import com.gemsrobotics.lib.utils.MathUtils;

import static com.gemsrobotics.lib.utils.MathUtils.epsilonEquals;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

public class BangBangController {
    // the setpoint of the controller
    protected double m_reference;
    // the allowed error in the controller
    protected double m_tolerance;
    // the current offset from the reference
    protected double m_error;

    public BangBangController() {
        m_reference = 0.0;
        m_tolerance = 0.0;
        m_error = Double.NaN;
    }

    public void reset() {
        m_error = Double.NaN;
    }

    public void setReference(final double reference) {
        m_reference = reference;
    }

    public void setTolerance(final double tolerance) {
        m_tolerance = abs(tolerance);
    }

    public double update(final double input) {
        m_error = m_reference - input;

        if (isOnTarget()) {
            return 0.0;
        } else {
            return signum(m_error);
        }
    }

    public boolean isOnTarget(final double tolerance) {
        return epsilonEquals(m_error, 0, tolerance);
    }

    public boolean isOnTarget() {
        return isOnTarget(m_tolerance);
    }
}
