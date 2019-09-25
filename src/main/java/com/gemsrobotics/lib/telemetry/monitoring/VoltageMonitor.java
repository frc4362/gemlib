package com.gemsrobotics.lib.telemetry.monitoring;

import java.util.Objects;

public final class VoltageMonitor extends Monitor {
    private static VoltageMonitor INSTANCE;

    public static VoltageMonitor getInstance() {
        if (Objects.isNull(INSTANCE)) {
            INSTANCE = new VoltageMonitor();
        }

        return INSTANCE;
    }

    private VoltageMonitor() {

    }

    @Override
    public void initialize() {

    }
}
