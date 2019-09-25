package com.gemsrobotics.lib.trajectory;

import com.gemsrobotics.lib.math.se2.State;
import com.google.gson.annotations.SerializedName;

public class TrajectoryPoint<S extends State<S>> {
    @SerializedName("index")
    protected final int m_index;
    @SerializedName("state")
    protected final S m_state;

    public TrajectoryPoint(final S state, int index) {
        m_state = state;
        m_index = index;
    }

    public S state() {
        return m_state;
    }

    public int index() {
        return m_index;
    }
}
