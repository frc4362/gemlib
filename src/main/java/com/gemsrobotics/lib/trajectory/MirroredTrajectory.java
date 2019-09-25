package com.gemsrobotics.lib.trajectory;

import com.gemsrobotics.lib.math.se2.RigidTransformWithCurvature;
import com.gemsrobotics.lib.trajectory.parameterization.TimedState;
import com.gemsrobotics.lib.trajectory.parameterization.TrajectoryUtils;

public class MirroredTrajectory {
	public final Trajectory<TimedState<RigidTransformWithCurvature>> left, right;

	public MirroredTrajectory(final Trajectory<TimedState<RigidTransformWithCurvature>> right) {
		this.right = right;
		this.left = TrajectoryUtils.mirrorTimed(right);
	}
}
