package com.gemsrobotics.lib.trajectory.parameterization;

import com.gemsrobotics.lib.subsystems.drivetrain.ChassisState;
import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDriveModel;
import com.gemsrobotics.lib.math.se2.ICurvature;
import com.gemsrobotics.lib.math.se2.IRigidTransform2d;
import com.google.gson.annotations.SerializedName;

public class DifferentialDriveDynamicsConstraint<S extends IRigidTransform2d<S> & ICurvature<S>> implements TimingConstraint<S> {
    protected transient final DifferentialDriveModel m_model;
    @SerializedName("voltageLimit")
    protected final double m_absoluteVoltageLimit;
    @SerializedName("isHighGear")
    protected final boolean m_isHighGear;

    public DifferentialDriveDynamicsConstraint(final DifferentialDriveModel model, final boolean isHighGear, final double absoluteVoltageLimit) {
        m_model = model;
        m_isHighGear = isHighGear;
        m_absoluteVoltageLimit = absoluteVoltageLimit;
    }

    @Override
    public double getMaxVelocity(final S state) {
        return m_model.calculateMaxVelocity(state.getCurvature(), m_absoluteVoltageLimit, m_isHighGear);
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(final S state, final double velocity) {
        // NOTE: units cancel on angular getVelocity.
        final var bounds = m_model.calculateMinMaxAcceleration(
                new ChassisState(velocity, state.getCurvature() * velocity),
                state.getCurvature(),
                m_absoluteVoltageLimit,
                m_isHighGear);

        return new MinMaxAcceleration(bounds.min, bounds.max);
    }
}
