package com.gemsrobotics.frc2022.subsystems;

import com.gemsrobotics.frc2022.ShotParameters;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.subsystems.Flywheel;
import com.gemsrobotics.lib.utils.FastDoubleToString;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Objects;
import java.util.Optional;

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
		SHOOTING
	}

	public enum SystemState {
		IDLE,
		INTAKING,
		OUTTAKING,
		SHOOTING
	}

	private final Intake m_intake;
	private final Uptake m_uptake;
	private final Flywheel m_shooterLower, m_shooterUpper;

	private final Timer m_stateChangeTimer, m_wantStateChangeTimer;
	private final PeriodicIO m_periodicIO;
	private boolean m_stateChanged;
	private WantedState m_stateWanted;
	private SystemState m_state;

	private Superstructure() {
		m_intake = Intake.getInstance();
		m_uptake = Uptake.getInstance();
		m_shooterLower = LowerWheel.getInstance();
		m_shooterUpper = UpperWheel.getInstance();

		m_stateChangeTimer = new Timer();
		m_wantStateChangeTimer = new Timer();

		m_periodicIO = new PeriodicIO();

		m_state = SystemState.IDLE;
		m_stateWanted = WantedState.IDLE;
	}

	private static class PeriodicIO {
		public Optional<ShotParameters> shotParameters;
	}

	public synchronized void setWantedState(final WantedState newState) {
		if (newState != m_stateWanted) {
			m_stateWanted = newState;
			m_wantStateChangeTimer.reset();
		}
	}

	@Override
	protected void readPeriodicInputs(double timestamp) {
		m_periodicIO.shotParameters = Optional.empty();
	}

	@Override
	protected void onStart(final double timestamp) {
		m_wantStateChangeTimer.start();
		m_stateChangeTimer.start();

		SmartDashboard.putNumber("Linear Velocity Reference", 4.0);
	}

	@Override
	protected void onUpdate(final double timestamp) {
		final SystemState newState;

//		SmartDashboard.putString("Wanted State", m_stateWanted.toString());
//		SmartDashboard.putString("Current State", m_state.toString());

		SmartDashboard.putString("Lower Linear Velocity", FastDoubleToString.format(m_shooterLower.getLinearVelocity()));
		SmartDashboard.putString("Upper Linear Velocity", FastDoubleToString.format(m_shooterUpper.getLinearVelocity()));

		switch (m_state) {
			case IDLE:
				newState = handleIdle();
				break;
			case INTAKING:
				newState = handleIntaking();
				break;
			case OUTTAKING:
				newState = handleOuttaking();
				break;
			case SHOOTING:
				newState = handleShooting();
				break;
			default:
				newState = SystemState.IDLE;
				break;
		}

		if (newState != m_state) {
			m_state = newState;
			m_stateChangeTimer.reset();
			m_stateChanged = true;
		} else {
			m_stateChanged = false;
		}
	}

	@Override
	protected void onStop(final double timestamp) {
		m_wantStateChangeTimer.stop();
		m_wantStateChangeTimer.reset();
		m_stateChangeTimer.stop();
		m_stateChangeTimer.reset();
	}

	private SystemState applyWantedState() {
		switch (m_stateWanted) {
			case INTAKING:
				return SystemState.INTAKING;
			case OUTTAKING:
				return SystemState.OUTTAKING;
			case SHOOTING:
				return SystemState.SHOOTING;
			case IDLE:
			default:
				return SystemState.IDLE;
		}
	}

	private SystemState handleIdle() {
		m_intake.setWantedState(Intake.State.RETRACTED);
		m_uptake.setWantedState(Uptake.State.NEUTRAL);
		setShootingVelocity(0.0);

		return applyWantedState();
	}

	private SystemState handleIntaking() {
		m_intake.setWantedState(Intake.State.INTAKING);
		m_uptake.setWantedState(Uptake.State.INTAKING);
		setShootingVelocity(0.0);

		return applyWantedState();
	}

	private SystemState handleOuttaking() {
		m_intake.setWantedState(Intake.State.OUTTAKING);
		m_uptake.setWantedState(Uptake.State.OUTTAKING);
		setShootingVelocity(0.0);

		return applyWantedState();
	}

	private SystemState handleShooting() {
		setShootingVelocity(SmartDashboard.getNumber("Linear Velocity Reference", 0.0));

		if (m_wantStateChangeTimer.get() > 0.5) {
			m_intake.setWantedState(Intake.State.RETRACTED);
			m_uptake.setWantedState(Uptake.State.FEEDING);
		} else {
			m_intake.setWantedState(Intake.State.RETRACTED);
			m_uptake.setWantedState(Uptake.State.NEUTRAL);
		}

		return applyWantedState();
	}

	public SystemState getSystemState() {
		return m_state;
	}

	private void setShootingVelocity(final double velocityMetersPerSecond) {
		m_shooterLower.setMetersPerSecond(velocityMetersPerSecond);
		m_shooterUpper.setMetersPerSecond(velocityMetersPerSecond);
	}

	@Override
	public void setSafeState() {

	}
}
