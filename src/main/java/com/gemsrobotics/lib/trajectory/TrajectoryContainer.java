package com.gemsrobotics.lib.trajectory;

import com.gemsrobotics.lib.math.se2.State;
import com.gemsrobotics.lib.trajectory.parameterization.TimedState;

public class TrajectoryContainer<S extends State<S>> {
    private final boolean m_usesHighGear;
    private final TrajectoryIterator<TimedState<S>> m_trajectory;

    public TrajectoryContainer(final boolean highGear, final Trajectory<TimedState<S>> trajectory) {
        m_usesHighGear = highGear;
        m_trajectory = new TrajectoryIterator<>(new TimedView<>(trajectory));
    }

    public boolean usesHighGear() {
        return m_usesHighGear;
    }

    public TrajectoryIterator<TimedState<S>> getTrajectory() {
        return m_trajectory;
    }
}
