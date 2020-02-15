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

	private Superstructure() {

	}

	public enum WantedState {
		IDLE,
		INTAKING,
		OUTTAKING,
		FEEDING,
		SHOOTING,
		CLIMBING,
		CONTROL_PANEL_ROTATION,
		CONTORL_PANEL_POSITION
	}

	public enum SystemState {
		IDLE,
		INTAKING, // includes serialization
		OUTTAKING,
		FEEDING,
		SPINNING_UP,
		SHOOTING,
		PREPARING_CLIMB,
		CLIMBING,
		CONTROL_PANEL_ROTATION,
		CONTROL_PANEL_POSITION
	}

	@Override
	protected void readPeriodicInputs(double timestamp) {

	}

	@Override
	protected void onCreate(double timestamp) {

	}

	@Override
	protected void onUpdate(double timestamp) {

	}

	@Override
	protected void onStop(double timestamp) {

	}

	@Override
	public void setSafeState() {

	}
}
