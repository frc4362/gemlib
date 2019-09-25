package com.gemsrobotics.lib.trajectory.parameterization;

import com.gemsrobotics.lib.math.se2.RigidTransformWithCurvature;

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

public class CentripetalAccelerationConstraint implements TimingConstraint<RigidTransformWithCurvature> {
    private final double m_maxCentripetalAcceleration;

    public CentripetalAccelerationConstraint(final double maxCentripetalAcceleration) {
        m_maxCentripetalAcceleration = maxCentripetalAcceleration;
    }

    @Override
    public double getMaxVelocity(final RigidTransformWithCurvature state) {
        return sqrt(abs(m_maxCentripetalAcceleration / state.getCurvature()));
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(final RigidTransformWithCurvature state, final double velocity) {
        return MinMaxAcceleration.NO_LIMITS;
    }
}
