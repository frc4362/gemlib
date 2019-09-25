package com.gemsrobotics.lib.trajectory;

import com.gemsrobotics.lib.math.se2.State;

import static com.gemsrobotics.lib.utils.MathUtils.epsilonEquals;

@SuppressWarnings("WeakerAccess")
public class DistanceView<S extends State<S>> implements TrajectoryView<S> {
    protected final Trajectory<S> m_trajectory;
    protected final double[] m_distances;

    public DistanceView(final Trajectory<S> trajectory) {
        m_trajectory = trajectory;
        m_distances = new double[m_trajectory.length()];

        m_distances[0] = 0.0;
        for (int i = 1; i < m_trajectory.length(); i++) {
            m_distances[i] = m_distances[i - 1] + m_trajectory.getState(i - 1).distance(m_trajectory.getState(i));
        }
    }

    @Override
    public TrajectorySamplePoint<S> sample(final double distance) {
        if (distance >= getLastInterpolant()) {
            return new TrajectorySamplePoint<>(m_trajectory.getPoint(m_trajectory.length() - 1));
        }

        if (distance <= 0.0) {
            return new TrajectorySamplePoint<>(m_trajectory.getPoint(0));
        }

        for (int i = 1; i < m_distances.length; i++) {
            final TrajectoryPoint<S> s = m_trajectory.getPoint(i);

            if (m_distances[i] >= distance) {
                final TrajectoryPoint<S> prev_s = m_trajectory.getPoint(i - 1);

                if (epsilonEquals(m_distances[i], m_distances[i - 1])) {
                    return new TrajectorySamplePoint<>(s);
                } else {
                    return new TrajectorySamplePoint<>(
                            prev_s.state().interpolate(
                                    s.state(),
                                    (distance - m_distances[i - 1]) / (m_distances[i] - m_distances[i - 1])),
                            i - 1, i
                    );
                }
            }
        }

        throw new RuntimeException();
    }

    @Override
    public double getLastInterpolant() {
        return m_distances[m_distances.length - 1];
    }

    @Override
    public double getFirstInterpolant() {
        return 0.0;
    }

    @Override
    public Trajectory<S> getTrajectory() {
        return m_trajectory;
    }
}
