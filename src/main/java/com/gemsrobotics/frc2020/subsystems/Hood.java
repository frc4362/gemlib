package com.gemsrobotics.frc2020.subsystems;

import com.gemsrobotics.frc2020.Constants;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.timing.ElapsedTimer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

import java.util.Objects;

public final class Hood extends Subsystem {
	private static final double STATE_CHANGE_TIME = 0.25;

	private static Hood INSTANCE;

	public static Hood getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Hood();
		}

		return INSTANCE;
	}

	private final Solenoid m_solenoid;
	private final ElapsedTimer m_stateChangedTimer;
	private final PeriodicIO m_periodicIO;

	private Hood() {
		m_solenoid = new Solenoid(Constants.HOOD_SOLENOID_PORT);
		m_stateChangedTimer = new ElapsedTimer(STATE_CHANGE_TIME);
		m_periodicIO = new PeriodicIO();
	}

	private static class PeriodicIO {
		public boolean desiredState = false;
	}

	@Override
	protected void readPeriodicInputs(final double timestamp) {
		m_periodicIO.desiredState = m_solenoid.get();
	}

	@Override
	protected void onStart(final double timestamp) {

	}

	@Override
	protected void onUpdate(final double timestamp) {

	}

	@Override
	protected void onStop(final double timestamp) {

	}

	@Override
	public void setSafeState() {

	}

	public synchronized void setDeployed(final boolean deployed) {
		m_solenoid.set(deployed);
		m_stateChangedTimer.reset();
	}

	public synchronized boolean wantsDeployed() {
		return m_periodicIO.desiredState;
	}

	public synchronized boolean isDeployed() {
		return m_periodicIO.desiredState == m_stateChangedTimer.hasElapsed();
	}
}
