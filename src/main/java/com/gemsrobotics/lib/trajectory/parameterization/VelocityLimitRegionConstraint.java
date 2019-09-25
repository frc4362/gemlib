package com.gemsrobotics.lib.trajectory.parameterization;

import com.gemsrobotics.lib.math.se2.ITranslation2d;
import com.gemsrobotics.lib.math.se2.Translation;
import com.google.gson.annotations.SerializedName;

public class VelocityLimitRegionConstraint<S extends ITranslation2d<S>> implements TimingConstraint<S> {
    @SerializedName("velocityMax")
    protected final double m_velocityLimit;
    @SerializedName("cornerMin")
    protected final Translation m_cornerMin;
    @SerializedName("cornerMax")
    protected final Translation m_cornerMax;

    public VelocityLimitRegionConstraint(
            final Translation cornerMin,
            final Translation cornerMax,
            final double velocityLimit
    ) {
        m_cornerMin = cornerMin;
        m_cornerMax = cornerMax;
        m_velocityLimit = velocityLimit;
    }

    @Override
    public double getMaxVelocity(final S state) {
        final Translation translation = state.getTranslation();

        if (translation.x() <= m_cornerMax.x()
            && translation.x() >= m_cornerMin.x()
            && translation.y() <= m_cornerMax.y()
            && translation.y() >= m_cornerMin.y()
        ) {
            return m_velocityLimit;
        }

        return Double.POSITIVE_INFINITY;
    }

    @Override
    public TimingConstraint.MinMaxAcceleration getMinMaxAcceleration(final S state, final double velocity) {
        return MinMaxAcceleration.NO_LIMITS;
    }
}
