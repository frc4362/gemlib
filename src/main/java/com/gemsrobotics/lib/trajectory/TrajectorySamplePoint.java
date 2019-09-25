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

    public TrajectorySamplePoint(final S state, final int index_floor, final int index_ceil) {
        m_state = state;
        m_indexFloor = index_floor;
        m_indexCeiling = index_ceil;
    }

    public TrajectorySamplePoint(final TrajectoryPoint<S> point) {
        m_state = point.state();
        m_indexFloor = m_indexCeiling = point.index();
    }

    public S state() {
        return m_state;
    }

    public int getIndexFloor() {
        return m_indexFloor;
    }

    public int getIndexCeiling() {
        return m_indexCeiling;
    }
}
