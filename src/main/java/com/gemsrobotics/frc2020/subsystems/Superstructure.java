package com.gemsrobotics.frc2020.subsystems;

import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.utils.Units;
import edu.wpi.first.wpilibj.Timer;
import io.github.oblarg.oblog.annotations.Log;

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
		FEEDING,
		WAITING_FOR_ALIGNMENT,
		WAITING_FOR_FLYWHEEL,
		SHOOTING,
		PREPARING_CLIMB,
		CLIMBING,
		CONTROL_PANEL_ROTATION,
		CONTROL_PANEL_POSITION
	}

	private final Chassis m_chassis;
	private final Shooter m_shooter;
	private final Turret m_turret;
	private final Hood m_hood;
	private final Hopper m_hopper;
	private final TargetState m_targetState;

	private final Timer m_stateChangeTimer, m_wantStateChangeTimer;

	private final PeriodicIO m_periodicIO;
	
	@Log.ToString
	private SystemState m_systemState;
	@Log.ToString
	private WantedState m_wantedState;
	private boolean m_stateChanged;

	private Superstructure() {
		m_chassis = Chassis.getInstance();
		m_shooter = Shooter.getInstance();
		m_turret = Turret.getInstance();
		m_hood = Hood.getInstance();
		m_hopper = Hopper.getInstance();
		m_targetState = TargetState.getInstance();

		m_stateChangeTimer = new Timer();
		m_wantStateChangeTimer = new Timer();
		
		m_periodicIO = new PeriodicIO();
	}

	private static class PeriodicIO {
		private RigidTransform currentPose = RigidTransform.identity();
		private Optional<TargetState.CachedTarget> target = Optional.empty();
	}

	public synchronized void setWantedState(final WantedState state) {
		m_wantedState = state;
		m_wantStateChangeTimer.reset();
	}

	@Override
	protected void readPeriodicInputs(final double timestamp) {
		m_periodicIO.currentPose = m_targetState.getFieldToVehicle(timestamp);
		m_periodicIO.target = m_targetState.getCachedFieldToTarget();
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
			case WAITING_FOR_ALIGNMENT:
				newState = handleWaitingForAlignment();
				break;
			case WAITING_FOR_FLYWHEEL:
				newState = handleWaitingForFlywheel();
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

	private SystemState handleWaitingForAlignment() {
		if (m_periodicIO.target.isPresent()) {
			final var target = m_periodicIO.target.get();
			setTurretTargetPoint(target.getFieldToTarget());

			final var isCloseShot = target.getTargetDistance() < 10.0;
			m_hood.setDeployed(isCloseShot);

			final var wheelSpeeds = m_chassis.getWheelProperty(MotorController::getVelocityLinearMetersPerSecond).map(Math::abs);

			if (m_turret.atReference(Rotation.degrees(0.15))
						&& m_hood.isDeployed() == m_hood.wantsDeployed()
						&& (wheelSpeeds.left + wheelSpeeds.right) < 0.03
						&& ((isCloseShot && target.getDriveDistanceSinceCapture() < 5.0) || target.getAge() < 0.25)
			) {
				return SystemState.WAITING_FOR_FLYWHEEL;
			}

			// note fallthrough
		}

		switch (m_wantedState) {
			case FEEDING:
				return SystemState.FEEDING;
			case IDLE:
			default:
				return SystemState.IDLE;
		}
	}

	private void handleWaitingForFlywheel() {

	}

	@Override
	public void setSafeState() {

	}

	private void setTurretTargetPoint(final Translation targetPoint) {
		m_periodicIO.currentPose.getTranslation().difference(targetPoint).direction().difference(m_turret.getRotation());
	}

	private void setTurretFieldRotation(final Rotation fieldRotation) {
		m_turret.setReferenceRotation(m_periodicIO.currentPose.getRotation().inverse().sum(fieldRotation));
	}
}
