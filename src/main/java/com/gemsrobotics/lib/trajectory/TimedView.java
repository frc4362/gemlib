package com.gemsrobotics.lib.trajectory;

import com.gemsrobotics.lib.math.se2.State;
import com.gemsrobotics.lib.trajectory.parameterization.TimedState;
import com.google.gson.annotations.SerializedName;

import static com.gemsrobotics.lib.utils.MathUtils.epsilonEquals;

public class TimedView<S extends State<S>> implements TrajectoryView<TimedState<S>> {
    @SerializedName("getTrajectory")
    protected final Trajectory<TimedState<S>> m_trajectory;
    @SerializedName("timeBegin")
    protected final double m_timeBegin;
    @SerializedName("timeEnd")
    protected final double m_timeEnd;

    public TimedView(Trajectory<TimedState<S>> trajectory) {
        m_trajectory = trajectory;
        m_timeBegin = m_trajectory.getState(0).t();
        m_timeEnd = m_trajectory.getState(m_trajectory.length() - 1).t();
    }

    @Override
    public double getFirstInterpolant() {
        return m_timeBegin;
    }

    @Override
    public double getLastInterpolant() {
        return m_timeEnd;
    }

    @Override
    public TrajectorySamplePoint<TimedState<S>> sample(double t) {
        if (t >= m_timeEnd) {
            return new TrajectorySamplePoint<>(m_trajectory.getPoint(m_trajectory.length() - 1));
        }

        if (t <= m_timeBegin) {
            return new TrajectorySamplePoint<>(m_trajectory.getPoint(0));
        }

        for (int i = 1; i < m_trajectory.length(); ++i) {
            final TrajectoryPoint<TimedState<S>> s = m_trajectory.getPoint(i);
            if (s.state().t() >= t) {
                final TrajectoryPoint<TimedState<S>> prev_s = m_trajectory.getPoint(i - 1);

                if (epsilonEquals(s.state().t(), prev_s.state().t())) {
                    return new TrajectorySamplePoint<>(s);
                }

                return new TrajectorySamplePoint<>(prev_s.state().interpolate(s.state(),
                        (t - prev_s.state().t()) / (s.state().t() - prev_s.state().t())), i - 1, i);
            }
        }

        throw new RuntimeException();
    }

    @Override
    public Trajectory<TimedState<S>> getTrajectory() {
        return m_trajectory;
    }
}
