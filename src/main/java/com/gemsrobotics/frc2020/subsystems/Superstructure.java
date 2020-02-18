package com.gemsrobotics.frc2020.subsystems;

import com.gemsrobotics.lib.structure.Subsystem;

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
		FEEDING,
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

	private SystemState m_systemState;
	private WantedState m_wantedState;

	@Override
	protected void readPeriodicInputs(final double timestamp) {

	}

	@Override
	protected void onStart(final double timestamp) {
		m_wantedState = WantedState.IDLE;
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
}
