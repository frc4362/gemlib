package com.gemsrobotics.lib.trajectory;

import com.gemsrobotics.lib.math.se2.RigidTransformWithCurvature;
import com.gemsrobotics.lib.trajectory.parameterization.TimedState;

public final class TrajectoryContainer {
    public final boolean usesHighGear;
    public final Trajectory<TimedState<RigidTransformWithCurvature>> trajectory;

    public TrajectoryContainer(final boolean highGear, final Trajectory<TimedState<RigidTransformWithCurvature>> traj) {
        usesHighGear = highGear;
        trajectory = traj;
    }
}
