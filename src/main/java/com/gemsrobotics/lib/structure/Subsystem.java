package com.gemsrobotics.lib.structure;

import com.gemsrobotics.lib.telemetry.reporting.Reportable;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public abstract class Subsystem
		extends edu.wpi.first.wpilibj.command.Subsystem
		implements Loggable, Reportable
{
    @Override
    public final String configureLogName() {
        return getName();
    }

    @Override
    protected void initDefaultCommand() {
        // subclasses may choose to use this functionality
    }

	public enum FaultedResponse {
		NONE,
		DISABLE_SUBSYSTEM,
		DISABLE_ROBOT
	}

    /**
     * Subclass is expected to announce and describe all faults, and return what the SystemManager
     * should do to handle the faulted subsystem.
     * @return The expected response from the SystemManager
     */
    protected synchronized FaultedResponse checkFaulted() {
        return FaultedResponse.NONE;
    }

    public abstract void setSafeState();

	private double m_dt = 0.0;
    private double m_lastUpdateTime = Double.NaN;
	private boolean m_isActive = false;

	protected synchronized final double dt() {
        return m_dt;
    }

    @Config.ToggleSwitch(defaultValue=false)
    public final void setActive(final boolean active) {
	    m_isActive = active;
    }

    public final boolean isInactive() {
        return !isActive();
    }

    @Log.BooleanBox(name="Enabled?")
    public final boolean isActive() {
	    return m_isActive;
    }

    protected abstract void readPeriodicInputs();

    protected abstract void onCreate(double timestamp);
    protected abstract void onEnable(double timestamp);
    protected abstract void onUpdate(double timestamp);
    protected abstract void onStop(double timestamp);

    protected synchronized final void updatePeriodicState(final double timestamp) {
        if (Double.isNaN(m_lastUpdateTime)) {
            m_lastUpdateTime = timestamp;
        }

        m_dt = m_lastUpdateTime - timestamp;
        m_lastUpdateTime = timestamp;

        readPeriodicInputs();
    }
}
