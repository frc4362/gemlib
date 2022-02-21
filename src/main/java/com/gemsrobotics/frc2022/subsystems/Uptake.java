package com.gemsrobotics.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.structure.Subsystem;

import java.util.Objects;

public final class Uptake extends Subsystem {
	private static final int
			MOTOR_PORT_TRANSFER = 6,
			MOTOR_PORT_UPTAKE = 7;

	private static Uptake INSTANCE;

	public static Uptake getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Uptake();
		}

		return INSTANCE;
	}

	private final MotorController<TalonFX>
			m_motorTransfer,
			m_motorUptake;
	private State m_wantedState;

	public enum State {
		NEUTRAL,
		INTAKING
	}

	private Uptake() {
		m_motorTransfer = MotorControllerFactory.createDefaultTalonFX(MOTOR_PORT_TRANSFER);
		m_motorTransfer.setInvertedOutput(false);

		m_motorUptake = MotorControllerFactory.createDefaultTalonFX(MOTOR_PORT_UPTAKE);
		m_motorTransfer.setInvertedOutput(false);
	}

	@Override
	protected void readPeriodicInputs(final double timestamp) {

	}

	@Override
	protected void onStart(final double timestamp) {
		m_motorTransfer.setNeutral();
		m_motorUptake.setNeutral();
	}

	@Override
	protected void onUpdate(final double timestamp) {
		if (m_wantedState == State.NEUTRAL) {
			m_motorTransfer.setNeutral();
			m_motorUptake.setNeutral();
		} else if (m_wantedState == State.INTAKING) {
			m_motorTransfer.setDutyCycle(0.5);
			m_motorUptake.setDutyCycle(0.5);
		}
	}

	@Override
	protected void onStop(final double timestamp) {
		m_motorTransfer.setNeutral();
		m_motorUptake.setNeutral();
	}

	@Override
	public void setSafeState() {
		m_motorTransfer.setNeutral();
		m_motorUptake.setNeutral();
	}

	public void setState(final State state) {
		m_wantedState = state;
	}
}
