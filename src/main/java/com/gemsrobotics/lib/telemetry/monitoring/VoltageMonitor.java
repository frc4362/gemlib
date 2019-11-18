package com.gemsrobotics.lib.telemetry.monitoring;

import com.gemsrobotics.lib.data.DigitalSignalTrigger;
import com.gemsrobotics.lib.telemetry.reporting.ReportingEndpoint.Event.Kind;
import com.gemsrobotics.lib.utils.FastDoubleToString;
import edu.wpi.first.wpilibj.RobotController;

import java.util.Objects;

public final class VoltageMonitor extends Monitor {
    private static final double BATTERY_OVERRUN_VOLTAGE = 13.0;
    private static final double BATTERY_UNDERRUN_VOLTAGE = 7.0;

    private static VoltageMonitor INSTANCE;

    public static VoltageMonitor getInstance() {
        if (Objects.isNull(INSTANCE)) {
            INSTANCE = new VoltageMonitor();
        }

        return INSTANCE;
    }

    private final DigitalSignalTrigger m_voltageOverflowTrigger, m_voltageUnderflowTrigger;

    private VoltageMonitor() {
        m_voltageOverflowTrigger = DigitalSignalTrigger.of(() -> RobotController.getBatteryVoltage() > BATTERY_OVERRUN_VOLTAGE);
        m_voltageUnderflowTrigger = DigitalSignalTrigger.of(() -> RobotController.getBatteryVoltage() < BATTERY_UNDERRUN_VOLTAGE);
    }

    @Override
    public void initializeDefaultBehaviour() {
        m_voltageOverflowTrigger.onRisingEdge(() -> report(Kind.WARNING, "Detected voltage overflow! Current voltage: "
                + FastDoubleToString.format(RobotController.getBatteryVoltage(), 1) + "V"));
        m_voltageUnderflowTrigger.onRisingEdge(() -> report(Kind.WARNING, "Detected voltage underflow! Current voltage: "
                + FastDoubleToString.format(RobotController.getBatteryVoltage(), 1) + "V"));
    }

    public DigitalSignalTrigger getVoltageOverrunTrigger() {
        return m_voltageOverflowTrigger;
    }

    public DigitalSignalTrigger getVoltageUnderrunTrigger() {
        return m_voltageUnderflowTrigger;
    }
}
