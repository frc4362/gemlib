package com.gemsrobotics.lib.trajectory.parameterization;

import com.gemsrobotics.lib.math.se2.State;

public interface TimingConstraint<S extends State<S>> {
    double getMaxVelocity(S state);

    MinMaxAcceleration getMinMaxAcceleration(S state, double velocity);

    class MinMaxAcceleration {
        protected final double m_accelerationMin;
        protected final double m_accelerationMax;

        public static MinMaxAcceleration NO_LIMITS = new MinMaxAcceleration();

        public MinMaxAcceleration() {
            // No limits.
            m_accelerationMin = Double.NEGATIVE_INFINITY;
            m_accelerationMax = Double.POSITIVE_INFINITY;
        }

        public MinMaxAcceleration(final double accelerationMin, final double accelerationMax) {
            m_accelerationMin = accelerationMin;
            m_accelerationMax = accelerationMax;
        }

        public double getAccelerationMin() {
            return m_accelerationMin;
        }

        public double getAccelerationMax() {
            return m_accelerationMax;
        }

        public boolean isValid() {
            return getAccelerationMin() <= getAccelerationMax();
        }

        @Override
        public String toString() {
            return "(" + getAccelerationMin() + ", " + getAccelerationMax() + ")";
        }
    }
}
