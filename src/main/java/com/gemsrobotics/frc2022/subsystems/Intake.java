package com.gemsrobotics.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.structure.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;

import java.util.Objects;

public final class Intake extends Subsystem {
	private static final int
			INTAKE_MOTOR_PORT = 5,
			INTAKE_SOLENOID_FORWARD_CHANNEL = 1,
			INTAKE_SOLENOID_REVERSE_CHANNEL = 2;
	private static final double EXTENSION_MOTOR_DELAY = 0.1;

	private static Intake INSTANCE;

	public static Intake getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Intake();
		}

		return INSTANCE;
	}

	public enum State {
		EXTENDED,
		RETRACTED
	}

	private final MotorController<TalonFX> m_motor;
	private final DoubleSolenoid m_extender;

	private State m_wantedState;
	private Timer m_wantStateChangeTimer;

	private Intake() {
		m_motor = MotorControllerFactory.createDefaultTalonFX(INTAKE_MOTOR_PORT);
		m_motor.setNeutralBehaviour(MotorController.NeutralBehaviour.COAST);
		m_motor.setInvertedOutput(false);

		m_extender = new DoubleSolenoid(
				PneumaticsModuleType.REVPH,
				INTAKE_SOLENOID_FORWARD_CHANNEL,
				INTAKE_SOLENOID_REVERSE_CHANNEL);

		m_wantStateChangeTimer = new Timer();
	}

	@Override
	protected synchronized void readPeriodicInputs(final double timestamp) {

	}

	@Override
	protected synchronized void onStart(final double timestamp) {
		m_extender.set(DoubleSolenoid.Value.kOff);

		m_wantStateChangeTimer.start();

		setWantedState(State.RETRACTED);
	}

	@Override
	protected synchronized void onUpdate(final double timestamp) {
		if (m_wantedState == State.EXTENDED) {
			m_extender.set(DoubleSolenoid.Value.kForward);

			if (m_wantStateChangeTimer.get() > EXTENSION_MOTOR_DELAY) {
				m_motor.setDutyCycle(-1.0);
			}
		} else {
			m_extender.set(DoubleSolenoid.Value.kReverse);
			m_motor.setDutyCycle(0.0);
		}
	}

	@Override
	protected synchronized void onStop(final double timestamp) {
		m_extender.set(DoubleSolenoid.Value.kReverse);
	}

	@Override
	public void setSafeState() {
		m_extender.set(DoubleSolenoid.Value.kOff);
	}

	public synchronized void setWantedState(final State state) {
		if (state != m_wantedState) {
			m_wantedState = state;
			m_wantStateChangeTimer.reset();
		}
	}
}
