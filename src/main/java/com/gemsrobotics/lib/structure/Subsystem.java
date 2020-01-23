package com.gemsrobotics.lib.structure;

import com.gemsrobotics.lib.telemetry.reporting.Reportable;
import com.gemsrobotics.lib.timing.DeltaTime;
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

    private final DeltaTime m_timer = new DeltaTime();
	private double m_dt = 0.0;
	private boolean m_isActive = true;

	// seconds
	public synchronized final double dt() {
        return m_dt;
    }

    @Config.ToggleSwitch(defaultValue=false)
    public final void setActive(final boolean active) {
	    m_isActive = active;
    }

    @Log.BooleanBox(name="Enabled?")
    public final boolean isActive() {
	    return m_isActive;
    }

    // should update the local PeriodicIO class's members
    protected abstract void readPeriodicInputs(double timestamp);

	// called once everything has been initialized
	// designed for things which must be initialized very late
	// or are dependant on other subsystems
    protected abstract void onCreate(double timestamp);
    // called every 10ms while the subsystem is enabled
    protected abstract void onUpdate(double timestamp);
    // called when the robot is disabled
    protected abstract void onStop(double timestamp);

    protected synchronized final void updatePeriodicState(final double timestamp) {
        m_dt = m_timer.update(timestamp);
        readPeriodicInputs(timestamp);
    }
}
