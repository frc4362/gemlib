package com.gemsrobotics.lib.trajectory;

import com.gemsrobotics.lib.math.se2.State;
import com.google.gson.annotations.SerializedName;

public class TrajectorySamplePoint<S extends State<S>> {
    @SerializedName("state")
    protected final S m_state;
    @SerializedName("indexFloor")
    protected final int m_indexFloor;
    @SerializedName("indexCeiling")
    protected final int m_indexCeiling;

    public TrajectorySamplePoint(final S state, final int indexFloor, final int indexCeiling) {
        m_state = state;
        m_indexFloor = indexFloor;
        m_indexCeiling = indexCeiling;
    }

    public TrajectorySamplePoint(final TrajectoryPoint<S> point) {
        m_state = point.state();
        m_indexFloor = m_indexCeiling = point.index();
    }

    public S getState() {
        return m_state;
    }

    public int getIndexFloor() {
        return m_indexFloor;
    }

    public int getIndexCeiling() {
        return m_indexCeiling;
    }
}
