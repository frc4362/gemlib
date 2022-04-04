package com.gemsrobotics.frc2022.subsystems.uptake;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.structure.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Objects;

public final class Uptake extends Subsystem {
	private static final int
			MOTOR_PORT_TRANSFER = 6,
			MOTOR_PORT_UPTAKE = 7,
			SENSOR_PORT_UPPER = 0,
			SENSOR_PORT_LOWER = 1;
	private static final int
			MOTOR_PWM_TRANSFER = 1,
			MOTOR_PWM_UPTAKE = 2;

	private static Uptake INSTANCE;

	public static Uptake getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Uptake();
		}

		return INSTANCE;
	}

//	private final MotorController<TalonFX>
//			m_motorTransfer,
//			m_motorUptake;
	 private final PWMTalonFX
	 		m_motorTransfer,
	 		m_motorUptake;
	private final DigitalInput
			m_sensorUpper,
			m_sensorLower;
	private final CargoColorObserver
			m_colorObserver;
	private final PeriodicIO m_periodicIO;
	private State m_wantedState;

	public enum State {
		NEUTRAL,
		INTAKING,
		OUTTAKING,
		FEEDING
	}

	private Uptake() {
//		m_motorTransfer = MotorControllerFactory.createDefaultTalonFX(MOTOR_PORT_TRANSFER);
//		m_motorTransfer.setInvertedOutput(true);
	 	m_motorTransfer = new PWMTalonFX(MOTOR_PWM_TRANSFER);
	 	m_motorTransfer.setInverted(true);

//		m_motorUptake = MotorControllerFactory.createDefaultTalonFX(MOTOR_PORT_UPTAKE);
//		m_motorUptake.setInvertedOutput(true);
		m_motorUptake = new PWMTalonFX(MOTOR_PWM_UPTAKE);
		m_motorUptake.setInverted(true);

		// true when beam is received
		m_sensorUpper = new DigitalInput(SENSOR_PORT_UPPER);
		m_sensorLower = new DigitalInput(SENSOR_PORT_LOWER);

		m_colorObserver = new LimelightColorObserver();

		m_periodicIO = new PeriodicIO();
	}

	private static class PeriodicIO {
		public boolean sensorUpper = false;
		public boolean sensorLower = false;
	}

	@Override
	protected void readPeriodicInputs(final double timestamp) {
		m_periodicIO.sensorUpper = m_sensorUpper.get();
		m_periodicIO.sensorLower = m_sensorLower.get();
	}

	@Override
	protected void onStart(final double timestamp) {
		m_motorTransfer.set(0.0);
		m_motorUptake.set(0.0);
//		m_motorTransfer.setNeutral();
//		m_motorUptake.setNeutral();
	}

	@Override
	protected void onUpdate(final double timestamp) {
		SmartDashboard.putBoolean("Bottom Broken", !m_periodicIO.sensorLower);
		SmartDashboard.putBoolean("Upper Broken", !m_periodicIO.sensorUpper);

		if (m_wantedState == State.NEUTRAL) {
			m_motorTransfer.set(0.0);
			m_motorUptake.set(0.0);
//			m_motorTransfer.setNeutral();
//			m_motorUptake.setNeutral();
		} else if (m_wantedState == State.INTAKING) {
			m_motorTransfer.set(!m_periodicIO.sensorLower && !m_periodicIO.sensorUpper ? 0.0 : 1.0);
			m_motorUptake.set(!m_periodicIO.sensorUpper ? 0.0 : 0.7);
//			m_motorTransfer.setDutyCycle(!m_periodicIO.sensorLower && !m_periodicIO.sensorUpper ? 0.0 : 1.0);
//			m_motorUptake.setDutyCycle(!m_periodicIO.sensorUpper ? 0.0 : 0.7);
		} else if (m_wantedState == State.FEEDING) {
			m_motorTransfer.set(1.0);
			m_motorUptake.set(0.7);
//			m_motorTransfer.setDutyCycle(1.0);
//			m_motorUptake.setDutyCycle(0.7);
		} else if (m_wantedState == State.OUTTAKING) {
			m_motorTransfer.set(-0.7);
			m_motorUptake.set(-0.5);
//			m_motorTransfer.setDutyCycle(-0.7);
//			m_motorUptake.setDutyCycle(-0.5);
		}
	}

	public int getBallCount() {
		if (!m_periodicIO.sensorUpper && !m_periodicIO.sensorLower) {
			return 2;
		} else if (!m_periodicIO.sensorLower && m_periodicIO.sensorUpper) {
			return 0;
		} else if (!m_periodicIO.sensorUpper && m_periodicIO.sensorLower) {
			return 1;
		} else {
			return 0;
		}
	}

	@Override
	protected void onStop(final double timestamp) {
		m_motorTransfer.set(0.0);
		m_motorUptake.set(0.0);
//		m_motorTransfer.setNeutral();
//		m_motorUptake.setNeutral();
	}

	@Override
	public void setSafeState() {
		m_motorTransfer.set(0.0);
		m_motorUptake.set(0.0);
//		m_motorTransfer.setNeutral();
//		m_motorUptake.setNeutral();
	}

	public void setWantedState(final State state) {
		m_wantedState = state;
	}

	public boolean isWrongCargoHeld() {
		return getBallCount() >= 1 && !m_colorObserver.isCargoOurs();
	}
}
