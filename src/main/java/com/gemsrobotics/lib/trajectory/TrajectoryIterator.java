package com.gemsrobotics.lib.trajectory;

import com.gemsrobotics.lib.math.se2.State;
import com.google.gson.annotations.SerializedName;

import static java.lang.Math.max;
import static java.lang.Math.min;

public class TrajectoryIterator<S extends State<S>> {
    @SerializedName("progress")
    protected double m_progress;
    @SerializedName("view")
    protected final TrajectoryView<S> m_view;
    @SerializedName("currentSample")
    protected transient TrajectorySamplePoint<S> m_currentSample;
    @SerializedName("name")
    protected String m_name;

    public TrajectoryIterator(final TrajectoryView<S> view) {
        m_view = view;

        // No effect if view is empty.
        m_currentSample = m_view.sample(m_view.getFirstInterpolant());
        m_progress = m_view.getFirstInterpolant();
    }

    public boolean isDone() {
        return getRemainingProgress() == 0.0;
    }

    public double getProgress() {
        return m_progress;
    }

    public double getRemainingProgress() {
        return max(0.0, m_view.getLastInterpolant() - m_progress);
    }

    public TrajectorySamplePoint<S> getSample() {
        return m_currentSample;
    }

    public S getState() {
        return getSample().state();
    }

    public TrajectorySamplePoint<S> advance(final double progressTime) {
        m_progress = max(m_view.getFirstInterpolant(), min(m_view.getLastInterpolant(), m_progress + progressTime));
        m_currentSample = m_view.sample(m_progress);
        return m_currentSample;
    }

    public TrajectorySamplePoint<S> preview(final double peekTime) {
        final double progress = max(m_view.getFirstInterpolant(), min(m_view.getLastInterpolant(), m_progress + peekTime));
        return m_view.sample(progress);
    }

    public Trajectory<S> getTrajectory() {
        return m_view.getTrajectory();
    }
}
