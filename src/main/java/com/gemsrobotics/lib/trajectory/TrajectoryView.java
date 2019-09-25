package com.gemsrobotics.lib.trajectory;

import com.gemsrobotics.lib.math.se2.State;

public interface TrajectoryView<S extends State<S>> {
    TrajectorySamplePoint<S> sample(double interpolant);

    double getFirstInterpolant();

    double getLastInterpolant();

    Trajectory<S> getTrajectory();
}
