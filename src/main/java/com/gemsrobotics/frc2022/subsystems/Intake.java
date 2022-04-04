package com.gemsrobotics.frc2022.subsystems;

import com.gemsrobotics.frc2022.PneumaticsContainer;
import com.gemsrobotics.lib.structure.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;

import java.util.Objects;

public final class Intake extends Subsystem {
	private static final int
			INTAKE_MOTOR_PORT = 5,
			INTAKE_SOLENOID_BUS_ID = 1,
			INTAKE_SOLENOID_FORWARD_CHANNEL = 8,
			INTAKE_SOLENOID_REVERSE_CHANNEL = 9;
	private static final double EXTENSION_MOTOR_DELAY = 0.3;
	private static final int MOTOR_PWM_INTAKE = 0;

	private static Intake INSTANCE;

	public static Intake getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Intake();
		}

		return INSTANCE;
	}

	public enum State {
		INTAKING,
		OUTTAKING,
		RETRACTED,
		EXTENDED
	}

//	private final MotorController<TalonFX> m_motor;
	private final PWMTalonFX m_motor;
	private final DoubleSolenoid m_extender;
	private final Timer m_intakeExtensionTimer;

	private State m_wantedState;

	private Intake() {
		m_motor = new PWMTalonFX(MOTOR_PWM_INTAKE);
		m_motor.setInverted(false);
//		m_motor = MotorControllerFactory.createDefaultTalonFX(INTAKE_MOTOR_PORT);
//		m_motor.setOpenLoopVoltageRampRate(0.05);
//		m_motor.setNeutralBehaviour(MotorController.NeutralBehaviour.COAST);
//		m_motor.setInvertedOutput(false);

		m_extender = PneumaticsContainer.getInstance().getIntakeSolenoid();

		m_intakeExtensionTimer = new Timer();
	}

	@Override
	protected void readPeriodicInputs(final double timestamp) {

	}

	@Override
	protected void onStart(final double timestamp) {
		m_extender.set(DoubleSolenoid.Value.kOff);
		m_intakeExtensionTimer.start();
		setWantedState(State.RETRACTED);
	}

	@Override
	protected void onUpdate(final double timestamp) {
		if (m_wantedState == State.INTAKING || m_wantedState == State.OUTTAKING) {
			m_extender.set(DoubleSolenoid.Value.kForward);

			if (m_intakeExtensionTimer.get() > EXTENSION_MOTOR_DELAY) {
				m_motor.set(m_wantedState == State.INTAKING ? 0.6 : -0.6);
//				m_motor.setDutyCycle(m_wantedState == State.INTAKING ? 0.6 : -0.6);
			}
		} else if (m_wantedState == State.EXTENDED) {
			m_extender.set(DoubleSolenoid.Value.kForward);
			m_motor.set(0.0);
//			m_motor.setDutyCycle(0.0);
		} else {
			m_extender.set(DoubleSolenoid.Value.kReverse);
			m_motor.set(0.0);
//			m_motor.setDutyCycle(0.0);
		}
	}

	@Override
	protected void onStop(final double timestamp) {
		m_extender.set(DoubleSolenoid.Value.kReverse);
	}

	@Override
	public void setSafeState() {
		m_extender.set(DoubleSolenoid.Value.kOff);
	}

	public void setWantedState(final State state) {
		if (state != m_wantedState) {
			if (m_wantedState == State.RETRACTED) {
				m_intakeExtensionTimer.reset();
			}

			m_wantedState = state;
		}
	}


}
