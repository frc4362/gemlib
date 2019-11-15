package com.gemsrobotics.lib.telemetry.monitoring;

import com.gemsrobotics.lib.telemetry.reporting.Reportable;

public abstract class Monitor implements Reportable {
    private boolean m_isActive = false;

    public final boolean isActive() {
        return m_isActive;
    }

    public final void doMonitoring() {
        m_isActive = true;
        initializeDefaultBehaviour();
    }

    public abstract void initializeDefaultBehaviour();
}
