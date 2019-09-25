package com.gemsrobotics.lib.telemetry.monitoring;

import com.gemsrobotics.lib.data.DigitalSignalTrigger;
import com.gemsrobotics.lib.property.CachedBoolean;
import com.gemsrobotics.lib.property.CachedValue;
import com.gemsrobotics.lib.telemetry.reporting.Reporter.Event.Kind;
import com.gemsrobotics.lib.utils.FastDoubleToString;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

import java.util.Map;
import java.util.Objects;

import static com.gemsrobotics.lib.telemetry.reporting.Reporter.Event.Kind.*;

public final class ConnectionMonitor extends Monitor {
    private static ConnectionMonitor INSTANCE;
    private final DriverStation m_ds;

    public static ConnectionMonitor getInstance() {
        if (Objects.isNull(INSTANCE)) {
            INSTANCE = new ConnectionMonitor();
        }

        return INSTANCE;
    }

    private final DigitalSignalTrigger
            m_connectedToField,
            m_connectedToDriverStation,
            m_isAutonomous,
            m_isTeleop,
            m_isEnabled;

    private boolean m_hasConnectedToFieldEver, m_hasConnectedToDriverStationEver;

    private ConnectionMonitor() {
        m_ds = DriverStation.getInstance();

        m_connectedToField = DigitalSignalTrigger.of(m_ds::isFMSAttached);
        m_connectedToDriverStation = DigitalSignalTrigger.of(m_ds::isFMSAttached);
        m_isAutonomous = DigitalSignalTrigger.of(m_ds::isAutonomous);
        m_isTeleop = DigitalSignalTrigger.of(m_ds::isOperatorControl);
        m_isEnabled = DigitalSignalTrigger.of(m_ds::isEnabled);

        m_hasConnectedToFieldEver = false;
        m_hasConnectedToDriverStationEver = false;
    }

    @Override
    public void initialize() {
        m_connectedToField.onRisingEdge(() -> {
            report(SYSTEM, "Field connected");

            if (!m_hasConnectedToFieldEver) {
                reportMatchInfo();
            }

            m_hasConnectedToFieldEver = true;
        });
        m_connectedToField.onFallingEdge(() -> report(SYSTEM, "Field disconnected"));

        m_connectedToDriverStation.onRisingEdge(() -> {
            report(SYSTEM, "Driver station connected.");
            m_hasConnectedToDriverStationEver = true;
        });
        m_connectedToDriverStation.onFallingEdge(() -> report(SYSTEM, "Driver station disconnected"));

        m_isAutonomous.onRisingEdge(() -> report(SYSTEM, "Entered autonomous."));
        m_isAutonomous.onFallingEdge(() -> report(SYSTEM, "Exited autonomous."));

        m_isTeleop.onRisingEdge(() -> report(SYSTEM, "Entered teleop."));
        m_isTeleop.onFallingEdge(() -> report(SYSTEM, "Exited teleop."));

        m_isEnabled.onRisingEdge(() -> report(SYSTEM, "Robot enabled."));
        m_isEnabled.onFallingEdge(() -> report(SYSTEM, "Robot disabled"));
    }

    public void reportMatchInfo() {
        report(SYSTEM, "Connected to field",
                Map.of("matchNumber", m_ds.getMatchNumber(),
                        "matchType", m_ds.getMatchType(),
                        "alliance", m_ds.getAlliance(),
                        "driverStation", m_ds.getLocation(),
                        "gameSpecificMessage", m_ds.getGameSpecificMessage(),
                        "batteryVoltage", FastDoubleToString.format(RobotController.getBatteryVoltage()),
                        "matchTime", FastDoubleToString.format(m_ds.getMatchTime(), 1)));
    }

    public boolean hasConnectedToField() {
        return m_hasConnectedToFieldEver;
    }

    public boolean hasConnectedToDriverStation() {
        return m_hasConnectedToDriverStationEver;
    }

    public DigitalSignalTrigger isConnectedToDriverStationTrigger() {
        return m_connectedToDriverStation;
    }

    public DigitalSignalTrigger isConnectedToFieldTrigger() {
        return m_connectedToField;
    }

    public DigitalSignalTrigger isAutonomousTrigger() {
        return m_isAutonomous;
    }

    public DigitalSignalTrigger isEnabledTrigger() {
        return m_isEnabled;
    }

    public DigitalSignalTrigger isTeleopTrigger() {
        return m_isTeleop;
    }
}
