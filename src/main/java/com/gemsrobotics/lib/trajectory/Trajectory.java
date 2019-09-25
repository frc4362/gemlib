package com.gemsrobotics.lib.trajectory;

import com.gemsrobotics.lib.math.se2.State;
import com.google.gson.annotations.SerializedName;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import static java.lang.Math.floor;

public class Trajectory<S extends State<S>> {
    @SerializedName("points")
    protected final List<TrajectoryPoint<S>> m_points;
    protected transient final IndexView m_indexView;

    /**
     * Create an empty getTrajectory.
     */
    public Trajectory() {
        m_indexView = new IndexView();
        m_points = new ArrayList<>();
    }

    /**
     * Create a getTrajectory from the given states and transforms.
     *
     * @param states The states of the getTrajectory.
     */
    public Trajectory(final List<S> states) {
        m_indexView = new IndexView();
        m_points = new ArrayList<>(states.size());

        for (int i = 0; i < states.size(); ++i) {
            m_points.add(new TrajectoryPoint<>(states.get(i), i));
        }
    }

    public boolean isEmpty() {
        return m_points.isEmpty();
    }

    public int length() {
        return m_points.size();
    }

    public TrajectoryPoint<S> getPoint(final int index) {
        return m_points.get(index);
    }

    public S getState(final int index) {
        return getPoint(index).state();
    }

    public S getFirstState() {
        return getState(0);
    }

    public S getLastState() {
        return getState(length() - 1);
    }

    public TrajectorySamplePoint<S> getInterpolated(final double index) {
        if (isEmpty()) {
            return null;
        } else if (index <= 0.0) {
            return new TrajectorySamplePoint<>(getPoint(0));
        } else if (index >= length() - 1) {
            return new TrajectorySamplePoint<>(getPoint(length() - 1));
        }

        final int i = (int) floor(index);
        final double frac = index - i;

        if (frac <= Double.MIN_VALUE) {
            return new TrajectorySamplePoint<>(getPoint(i));
        } else if (frac >= 1.0 - Double.MIN_VALUE) {
            return new TrajectorySamplePoint<>(getPoint(i + 1));
        } else {
            return new TrajectorySamplePoint<>(getState(i).interpolate(getState(i + 1), frac), i, i + 1);
        }
    }

    public void forEach(final Consumer<S> action) {
        for (int i = 0; i < length(); i++) {
            action.accept(getState(i));
        }
    }

    public IndexView getIndexView() {
        return m_indexView;
    }

    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder();

        builder.append('[');

        for (int i = 0; i < length(); ++i) {
            builder.append(i);
            builder.append(": ");
            builder.append(getState(i));
            builder.append(System.lineSeparator());
        }

        builder.append(']');

        return builder.toString();
    }

    public List<TrajectoryPoint<S>> getPoints() {
        return m_points;
    }

    public class IndexView implements TrajectoryView<S> {
        @Override
        public TrajectorySamplePoint<S> sample(double index) {
            return Trajectory.this.getInterpolated(index);
        }

        @Override
        public double getLastInterpolant() {
            return Math.max(0.0, Trajectory.this.length() - 1);
        }

        @Override
        public double getFirstInterpolant() {
            return 0.0;
        }

        @Override
        public Trajectory<S> getTrajectory() {
            return Trajectory.this;
        }
    }
}
