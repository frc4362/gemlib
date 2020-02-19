package com.gemsrobotics.frc2020.subsystems;

import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.timing.ElapsedTimer;
import edu.wpi.first.wpilibj.Timer;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.Objects;

public final class Superstructure extends Subsystem {
	private static Superstructure INSTANCE;

	public static Superstructure getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Superstructure();
		}

		return INSTANCE;
	}

	public enum WantedState {
		IDLE,
		INTAKING,
		OUTTAKING,
		SHOOTING,
		CLIMBING,
		CONTROL_PANEL_ROTATION,
		CONTROL_PANEL_POSITION
	}

	public enum SystemState {
		IDLE,
		INTAKING, // includes serialization
		OUTTAKING,
		WAITING_FOR_ALIGNMENT,
		WAITING_FOR_FLYWHEEL,
		SHOOTING,
		PREPARING_CLIMB,
		CLIMBING,
		CONTROL_PANEL_ROTATION,
		CONTROL_PANEL_POSITION
	}

	private final Chassis m_chassis;
	private final Hopper m_hopper;

	private final Timer m_stateChangeTimer, m_wantStateChangeTimer;

	@Log.ToString
	private SystemState m_systemState;
	@Log.ToString
	private WantedState m_wantedState;
	private boolean m_stateChanged;

	private Superstructure() {
		m_chassis = Chassis.getInstance();
		m_hopper = Hopper.getInstance();

		m_stateChangeTimer = new Timer();
		m_wantStateChangeTimer = new Timer();
	}

	public synchronized void setWantedState(final WantedState state) {
		m_wantedState = state;
		m_wantStateChangeTimer.reset();
	}

	@Override
	protected void readPeriodicInputs(final double timestamp) {

	}

	@Override
	protected void onStart(final double timestamp) {
		m_wantStateChangeTimer.start();
		m_stateChangeTimer.start();
		setWantedState(WantedState.IDLE);
	}

	@Override
	protected void onUpdate(final double timestamp) {
		final SystemState newState;

		switch (m_systemState) {
			case IDLE:
				newState = handleIdle();
				break;
			default:
				newState = SystemState.IDLE;
				break;
		}

		if (newState != m_systemState) {
			m_systemState = newState;
			m_stateChangeTimer.reset();
			m_stateChanged = true;
		} else {
			m_stateChanged = false;
		}
	}

	@Override
	protected void onStop(final double timestamp) {

	}

	private SystemState handleIdle() {
		m_hopper.setDisabled();
		
		switch (m_wantedState) {
			default:
				return SystemState.IDLE;
		}
	}

	@Override
	public void setSafeState() {

	}
}
