package com.gemsrobotics.lib.controls;

import static com.gemsrobotics.lib.utils.MathUtils.epsilonEquals;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

public final class BangBangController extends FeedbackController {
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

    @Override
    public void reset() {
        m_error = Double.NaN;
    }

    @Override
    public void setReference(final double reference) {
        m_reference = reference;
    }

    @Override
    public double getReference() {
        return m_reference;
    }

    @Override
    public void setTolerance(final double tolerance) {
        m_tolerance = abs(tolerance);
    }

    @Override
    public double getTolerance() {
        return m_tolerance;
    }

    @Override
    public double update(final double dt, final double input) {
        m_error = m_reference - input;

        if (atReference()) {
            return 0.0;
        } else {
            return signum(m_error);
        }
    }

    @Override
    public boolean atReference(final double tolerance) {
        return epsilonEquals(m_error, 0, tolerance);
    }
}
