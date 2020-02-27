package com.gemsrobotics.frc2020.subsystems;

import com.gemsrobotics.frc2020.Constants;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.utils.Units;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import io.github.oblarg.oblog.annotations.Log;

import java.util.List;
import java.util.Objects;
import java.util.Optional;

import static java.lang.Math.*;

public final class Superstructure extends Subsystem {
	public static final double TARGET_CACHE_DISTANCE = 5.0;
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
		CLIMBING
//		CONTROL_PANEL_ROTATION,
//		CONTROL_PANEL_POSITION
	}

	public enum SystemState {
		IDLE,
		INTAKING,
		OUTTAKING,
		FEEDING,
		WAITING_FOR_ALIGNMENT,
		WAITING_FOR_FLYWHEEL,
		SHOOTING,
		PREPARING_CLIMB,
		CLIMBING
//		CONTROL_PANEL_ROTATION,
//		CONTROL_PANEL_POSITION
	}

	private final Chassis m_chassis;
	private final Shooter m_shooter;
	private final Turret m_turret;
	private final Hood m_hood;
	private final Hopper m_hopper;
	private final RobotState m_targetState;
	private final List<Intake> m_intakes;
	private final Inventory m_inventory;

	private final Solenoid m_intakeDeployer;

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
		m_targetState = RobotState.getInstance();
		m_intakes = Intake.getAll();
		m_inventory = new Inventory();

		m_intakeDeployer = new Solenoid(Constants.INTAKE_SOLENOID_PORT);

		m_stateChangeTimer = new Timer();
		m_wantStateChangeTimer = new Timer();

		m_periodicIO = new PeriodicIO();
	}

	private static class PeriodicIO {
		private RigidTransform vehiclePose = RigidTransform.identity();
		private RigidTransform turretPose = RigidTransform.identity();
		private Optional<RobotState.CachedTarget> target = Optional.empty();
	}

	public synchronized void setWantedState(final WantedState state) {
		m_wantedState = state;
		m_wantStateChangeTimer.reset();
	}

	@Override
	protected synchronized void readPeriodicInputs(final double timestamp) {
		m_periodicIO.vehiclePose = m_targetState.getFieldToVehicle(timestamp);
		m_periodicIO.turretPose = m_targetState.getFieldToTurret(timestamp);
		m_periodicIO.target = m_targetState.getCachedFieldToTarget();
	}

	@Override
	protected synchronized void onStart(final double timestamp) {
		m_wantStateChangeTimer.start();
		m_stateChangeTimer.start();
		setWantedState(WantedState.IDLE);
	}

	@Override
	protected synchronized void onUpdate(final double timestamp) {
		final SystemState newState;

		if (m_hopper.atRest()) {
			m_inventory.setRotations(m_hopper.getRotations());
		}

		switch (m_systemState) {
			case IDLE:
				newState = handleIdle();
				break;
			case INTAKING:
				newState = handleIntaking();
				break;
			case OUTTAKING:
				newState = handleOuttaking();
				break;
			case WAITING_FOR_ALIGNMENT:
				newState = handleWaitingForAlignment();
				break;
			case WAITING_FOR_FLYWHEEL:
				newState = handleWaitingForFlywheel();
				break;
			case SHOOTING:
				newState = handleShooting();
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
	protected synchronized void onStop(final double timestamp) {
		m_wantStateChangeTimer.stop();
		m_wantStateChangeTimer.reset();
		m_stateChangeTimer.stop();
		m_stateChangeTimer.reset();
	}

	private SystemState handleIdle() {
		m_turret.setDisabled();
		m_hood.setDeployed(false);
		m_shooter.setRPM(0.0);
		m_intakeDeployer.set(false);

		switch (m_wantedState) {
			case INTAKING:
				return SystemState.INTAKING;
			case OUTTAKING:
				return SystemState.OUTTAKING;
			case SHOOTING:
				return SystemState.WAITING_FOR_ALIGNMENT;
			case IDLE:
			default:
				return SystemState.IDLE;
		}
	}

	private SystemState handleIntaking() {
		m_turret.setDisabled();
		m_hood.setDeployed(false);
		m_shooter.setRPM(0.0);
		m_intakeDeployer.set(true);

		if (m_wantStateChangeTimer.get() > 0.25) {
			final var wheelSpeeds = m_chassis.getWheelProperty(MotorController::getVelocityLinearMetersPerSecond).map(Math::abs);
			final var intakeVelocityMetersPerSecond = max(wheelSpeeds.left, wheelSpeeds.right) + 3.0;

			maybeRunIntakes(intakeVelocityMetersPerSecond);
		}

		switch (m_wantedState) {
			case INTAKING:
				return SystemState.INTAKING;
			case OUTTAKING:
				return SystemState.OUTTAKING;
			case SHOOTING:
				return SystemState.WAITING_FOR_ALIGNMENT;
			case IDLE:
			default:
				return SystemState.IDLE;
		}
	}

	private SystemState handleOuttaking() {
		m_turret.setDisabled();
		m_hood.setDeployed(false);
		m_shooter.setRPM(0.0);
		m_intakeDeployer.set(true);

		if (m_wantStateChangeTimer.get() > 0.25) {
			maybeRunIntakes(-3.0);
			m_intakes.forEach(Intake::clear);
		}

		if (m_wantStateChangeTimer.get() < 3.0) {
			return SystemState.INTAKING;
		}

		switch (m_wantedState) {
			case INTAKING:
				return SystemState.INTAKING;
			case OUTTAKING:
				return SystemState.OUTTAKING;
			case SHOOTING:
				return SystemState.WAITING_FOR_ALIGNMENT;
			case IDLE:
			default:
				return SystemState.IDLE;
		}
	}

	// TODO
//	private SystemState handleFeeding() {
//		if (m_stateChanged) {
//
//		}
//	}

	private SystemState handleWaitingForAlignment() {
		m_intakeDeployer.set(false);

		if (m_stateChanged && m_inventory.getFilledChamberCount() > 0) {
			final var loadingChamber = m_inventory.getOptimalShootingChamber();
			final var currentChamber = m_inventory.getNearestChamber(Inventory.Location.SHOOTER);
			m_hopper.rotate(currentChamber.getDistance(loadingChamber));
		}

		if (m_periodicIO.target.isPresent()) {
			final var target = m_periodicIO.target.get();
			final var goal = target.getOptimalGoal();
			final var outerDistance = target.getFieldToOuterGoal().distance(m_periodicIO.turretPose.getTranslation());

			m_hood.setDeployed(outerDistance < Constants.CLOSE_SHOT_RANGE_METERS);
			setTurretTargetPoint(goal);

			if (isAligned(target, Rotation.degrees(1.5), true) && m_inventory.getFilledChamberCount() > 0) {
				return SystemState.WAITING_FOR_FLYWHEEL;
			}
		} else {
			setTurretFieldRotation(Rotation.degrees(0));
		}

		switch (m_wantedState) {
			case INTAKING:
				return SystemState.INTAKING;
			case OUTTAKING:
				return SystemState.OUTTAKING;
			case SHOOTING:
				return SystemState.WAITING_FOR_ALIGNMENT;
			case IDLE:
			default:
				return SystemState.IDLE;
		}
	}

	private SystemState handleWaitingForFlywheel() {
		final var target = m_periodicIO.target.get();
		final var targetDistance = target.getOptimalGoal().distance(m_periodicIO.turretPose.getTranslation());

		final double targetRPM;

		if (targetDistance < Constants.WALL_SHOOTING_RPM) {
			targetRPM = Constants.WALL_SHOOTING_RPM;
		} else {
			targetRPM = Constants.SHOOTER_RANGE_REGRESSION.predict(targetDistance);
		}

		m_shooter.setRPM(targetRPM);

		if (!isAligned(m_periodicIO.target.get(), Rotation.degrees(3.0), false)) {
			return SystemState.WAITING_FOR_ALIGNMENT;
		}

		if (m_turret.atReference(Rotation.degrees(0.15)) && isReadyToFire()) {
			return SystemState.SHOOTING;
		}

		switch (m_wantedState) {
			case INTAKING:
				return SystemState.INTAKING;
			case OUTTAKING:
				return SystemState.OUTTAKING;
			case SHOOTING:
				return SystemState.WAITING_FOR_FLYWHEEL;
			case IDLE:
			default:
				return SystemState.IDLE;
		}
	}

	public SystemState handleShooting() {
		if (m_stateChanged) {
			m_hopper.rotate(6);
		}

		if (!m_hopper.atRest()) {
			return SystemState.SHOOTING;
		}

		switch (m_wantedState) {
			case INTAKING:
				return SystemState.INTAKING;
			case OUTTAKING:
				return SystemState.OUTTAKING;
			case SHOOTING:
				return SystemState.WAITING_FOR_ALIGNMENT;
			case IDLE:
			default:
				return SystemState.IDLE;
		}
	}

	@Override
	public void setSafeState() {

	}

	private boolean isAligned(final RobotState.CachedTarget target, final Rotation tolerance, final boolean allowDeadspot) {
		final var wheelSpeeds = m_chassis.getWheelProperty(MotorController::getVelocityLinearMetersPerSecond).map(Math::abs);
		final var outerDistance = target.getFieldToOuterGoal().distance(m_periodicIO.turretPose.getTranslation());
		final var isCloseShot = outerDistance < Constants.CLOSE_SHOT_RANGE_METERS;
		final var isFarShot = allowDeadspot && outerDistance > (Constants.CLOSE_SHOT_RANGE_METERS + Units.feet2Meters(1.5));

		return m_turret.atReference(tolerance)
			   && (wheelSpeeds.left + wheelSpeeds.right) < 0.03
			   && ((isCloseShot && target.getDriveDistanceSinceCapture() < TARGET_CACHE_DISTANCE) || (isFarShot && target.getAge() < 0.25));
	}

	private boolean isReadyToFire() {
		return m_shooter.atReference()
			   && m_hopper.atRest()
			   && m_hood.isDeployed() == m_hood.wantsDeployed();
	}

	private void setTurretTargetPoint(final Translation targetPoint) {
		m_turret.setReferenceRotation(m_periodicIO.turretPose
											      .getTranslation()
											      .difference(targetPoint)
											      .direction()
											      .difference(m_periodicIO.turretPose.getRotation()));
	}

	private void setTurretFieldRotation(final Rotation fieldRotation) {
		m_turret.setReferenceRotation(fieldRotation.difference(m_periodicIO.vehiclePose.getRotation()));
	}

	private void maybeRunIntakes(final double speedMetersPerSecond) {
		m_intakes.forEach(intake -> {
			if (intake.hasBall()) {
				intake.setVelocity(0.0);
			} else {
				intake.setVelocity(speedMetersPerSecond);
			}
		});
	}
}
