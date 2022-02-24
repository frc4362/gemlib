package com.gemsrobotics.frc2022.subsystems;

import com.gemsrobotics.frc2022.ShotParameters;
import com.gemsrobotics.lib.structure.Subsystem;
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
		INTAKING
	}

	public enum SystemState {
		IDLE,
		INTAKING
	}

	private final Timer m_stateChangeTimer, m_wantStateChangeTimer;
	private final PeriodicIO m_periodicIO;
	private boolean m_stateChanged;
	private WantedState m_stateWanted;
	private SystemState m_state;

	private GreyTTurret m_greytestTurret;
	private Chassis m_chassis;
	private Intake m_intake;
	private Uptake m_uptake;
	private ClimbElevator m_climbElevator;

	private Superstructure() {
		m_chassis = m_chassis.getInstance();
		m_greytestTurret = m_greytestTurret.getInstance();
		m_intake = m_intake.getInstance();
		m_uptake = m_uptake.getInstance();
		m_climbElevator = m_climbElevator.getInstance();

		m_stateChangeTimer = new Timer();
		m_wantStateChangeTimer = new Timer();

		m_periodicIO = new PeriodicIO();
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
	}

	@Override
	protected void onUpdate(final double timestamp) {
		final SystemState newState;

		SmartDashboard.putString("Wanted State", m_stateWanted.toString());
		SmartDashboard.putString("Current State", m_state.toString());

		switch (m_state) {
			case IDLE:
				newState = handleIdle();
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
			case IDLE:
			default:
				return SystemState.IDLE;
		}
	}

	private SystemState handleIdle() {
		return applyWantedState();
	}

	public SystemState getSystemState() {
		return m_state;
	}

	@Override
	public void setSafeState() {

	}
}
