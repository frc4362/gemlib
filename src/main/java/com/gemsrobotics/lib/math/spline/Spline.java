package com.gemsrobotics.lib.math.spline;

import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.RigidTransformWithCurvature;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;

public abstract class Spline {
    public abstract Translation getPoint(double t);

    public abstract Rotation getHeading(double t);

    public abstract double getCurvature(double t);

    // dk/ds
    public abstract double getDCurvature(double t);

    // ds/dt
    public abstract double getVelocity(double t);

    public RigidTransform getRigidTransform2d(double t) {
        return new RigidTransform(getPoint(t), getHeading(t));
    }

    public RigidTransformWithCurvature getRigidTransform2dWithCurvature(double t) {
        return new RigidTransformWithCurvature(
                getRigidTransform2d(t),
                getCurvature(t),
                getDCurvature(t) / getVelocity(t));
    }
}
