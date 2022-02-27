package com.gemsrobotics.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.structure.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Objects;

public final class Uptake extends Subsystem {
	private static final int
			MOTOR_PORT_TRANSFER = 6,
			MOTOR_PORT_UPTAKE = 7,
			SENSOR_PORT = 0;

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
	private final DigitalInput m_sensor;
	private final PeriodicIO m_periodicIO;
	private State m_wantedState;

	public enum State {
		NEUTRAL,
		INTAKING,
		OUTTAKING,
		FEEDING
	}

	private Uptake() {
		m_motorTransfer = MotorControllerFactory.createDefaultTalonFX(MOTOR_PORT_TRANSFER);
		m_motorTransfer.setInvertedOutput(true);

		m_motorUptake = MotorControllerFactory.createDefaultTalonFX(MOTOR_PORT_UPTAKE);
		m_motorUptake.setInvertedOutput(true);

		// true when beam is received
		m_sensor = new DigitalInput(0);

		m_periodicIO = new PeriodicIO();
	}

	private static class PeriodicIO {
		public boolean sensor = false;
	}

	@Override
	protected void readPeriodicInputs(final double timestamp) {
		m_periodicIO.sensor = m_sensor.get();
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
			m_motorUptake.setDutyCycle(!m_periodicIO.sensor ? 0.0 : 0.5);
		} else if (m_wantedState == State.FEEDING) {
			m_motorTransfer.setDutyCycle(0.5);
			m_motorUptake.setDutyCycle(0.5);
		} else if (m_wantedState == State.OUTTAKING) {
			m_motorTransfer.setDutyCycle(-0.7);
//			m_motorUptake.setDutyCycle(-0.5);
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

	public void setWantedState(final State state) {
		m_wantedState = state;
	}
}
