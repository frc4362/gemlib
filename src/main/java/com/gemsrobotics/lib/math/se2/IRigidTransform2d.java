package com.gemsrobotics.lib.math.se2;

public interface IRigidTransform2d<S> extends IRotation2d<S>, ITranslation2d<S> {
    RigidTransform getRigidTransform();
    S transformBy(RigidTransform transform);
    S mirror();
}
