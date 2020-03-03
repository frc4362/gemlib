package com.gemsrobotics.frc2020.subsystems;

import com.gemsrobotics.frc2020.Constants;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.subsystems.Limelight;
import com.gemsrobotics.lib.utils.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.annotations.Log;

import java.awt.*;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

import static java.lang.Math.*;

public final class Superstructure extends Subsystem {
	public static final double TARGET_CACHE_DISTANCE = 5.0;
	public static final double STOP_SHOOT_TIME_SECONDS = 0.250;
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
		CLIMB_EXTEND,
		CLIMB_RETRACT
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
		CLIMB_EXTEND,
		CLIMB_RETRACT
//		CONTROL_PANEL_ROTATION,
//		CONTROL_PANEL_POSITION
	}

	private final LEDs m_leds;
	private final TargetServer m_targetServer;
	private final Chassis m_chassis;
	private final Shooter m_shooter;
	private final Turret m_turret;
	private final Hood m_hood;
	private final Hopper m_hopper;
	private final RobotState m_targetState;
	// TODO
	private List<Intake> m_intakes;
	private final Inventory m_inventory;

	// TODO
	private Solenoid m_intakeDeployer, m_kicker;
	private DoubleSolenoid m_pto;

	private final Timer m_stateChangeTimer, m_wantStateChangeTimer;

	private final PeriodicIO m_periodicIO;

	@Log.ToString
	private SystemState m_systemState;
	@Log.ToString
	private WantedState m_wantedState;
	private boolean m_stateChanged;

	private Superstructure() {
		m_leds = LEDs.getInstance();
		m_targetServer = TargetServer.getInstance();
		m_chassis = Chassis.getInstance();
		m_shooter = Shooter.getInstance();
		m_turret = Turret.getInstance();
		m_hood = Hood.getInstance();
		m_hopper = Hopper.getInstance();
		m_targetState = RobotState.getInstance();
//		m_intakes = Intake.getAll();
		m_inventory = new Inventory();

		m_kicker = new Solenoid(Constants.KICKER_SOLENOID_PORT);
//		m_intakeDeployer = new Solenoid(Constants.INTAKE_SOLENOID_PORT);
		m_pto = new DoubleSolenoid(Constants.PTO_SOLENOID_PORT[0], Constants.PTO_SOLENOID_PORT[1]);

		m_stateChangeTimer = new Timer();
		m_wantStateChangeTimer = new Timer();

		m_systemState = SystemState.IDLE;

		m_periodicIO = new PeriodicIO();
	}

	private static class PeriodicIO {
		private RigidTransform vehiclePose = RigidTransform.identity();
		private RigidTransform turretPose = RigidTransform.identity();
		private Optional<RobotState.CachedTarget> target = Optional.empty();
	}

	public synchronized void setWantedState(final WantedState newState) {
		if (newState != m_wantedState) {
			m_wantedState = newState;
			m_wantStateChangeTimer.reset();
		}
	}

	public synchronized SystemState getSystemState() {
		return m_systemState;
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

		SmartDashboard.putString("Wanted State", m_wantedState.toString());
		SmartDashboard.putString("Current State", m_systemState.toString());

		// TODO
//		if (m_hopper.atRest()) {
//			m_inventory.setRotations(m_hopper.getRotations());
//		}

		final var facingTarget = m_periodicIO.turretPose.getRotation().getNearestPole().equals(Rotation.degrees(0));
		m_targetServer.setLEDMode(m_systemState != SystemState.CLIMB_EXTEND && m_systemState != SystemState.CLIMB_RETRACT && facingTarget ? Limelight.LEDMode.ON : Limelight.LEDMode.OFF);

		switch (m_systemState) {
			case IDLE:
				newState = handleIdle();
				break;
			case CLIMB_EXTEND:
				newState = handleClimbExtend();
				break;
			case CLIMB_RETRACT:
				newState = handleClimbRetract();
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
		m_kicker.set(false);
		m_turret.setDisabled();
		m_hood.setDeployed(false);
		m_shooter.setDisabled();

		m_leds.setColor(Color.BLACK, 0.0f);
		m_pto.set(DoubleSolenoid.Value.kReverse);

		switch (m_wantedState) {
			case CLIMB_EXTEND:
				return SystemState.CLIMB_EXTEND;
			case CLIMB_RETRACT:
				return SystemState.CLIMB_RETRACT;
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

	private SystemState handleClimbExtend() {
		m_turret.setDisabled();
		m_hood.setDeployed(false);
		m_hopper.setDisabled();
		m_shooter.setDisabled();

		m_leds.setOn(true);
		m_leds.setColorWave(Color.RED, 0.75f);

		m_pto.set(DoubleSolenoid.Value.kForward);

		switch (m_wantedState) {
			case CLIMB_EXTEND:
				return SystemState.CLIMB_EXTEND;
			case CLIMB_RETRACT:
				return SystemState.CLIMB_RETRACT;
			default:
				return SystemState.IDLE;
		}
	}

	private SystemState handleClimbRetract() {
		m_turret.setDisabled();
		m_hood.setDeployed(false);
		m_hopper.setDisabled();
		m_shooter.setDisabled();

		m_leds.setOn(true);
		m_leds.setColorWave(Color.RED, 0.15f);

		m_pto.set(DoubleSolenoid.Value.kForward);

		switch (m_wantedState) {
			case CLIMB_RETRACT:
				return SystemState.CLIMB_RETRACT;
			default:
				return SystemState.IDLE;
		}
	}

	private SystemState handleIntaking() {
		m_turret.setDisabled();
		m_hood.setDeployed(false);
		m_shooter.setDisabled();

		if (m_wantStateChangeTimer.get() > 0.25) {
			final var wheelSpeeds = m_chassis.getWheelProperty(MotorController::getVelocityLinearMetersPerSecond).map(Math::abs);
			final var intakeVelocityMetersPerSecond = max(wheelSpeeds.left, wheelSpeeds.right) + 3.0;

			maybeRunIntakes(intakeVelocityMetersPerSecond);
		}

		switch (m_wantedState) {
			case CLIMB_EXTEND:
				return SystemState.CLIMB_EXTEND;
			case CLIMB_RETRACT:
				return SystemState.CLIMB_RETRACT;
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

		if (m_wantStateChangeTimer.get() > 0.25) {
			maybeRunIntakes(-3.0);
			m_intakes.forEach(Intake::clear);
		}

		if (m_wantStateChangeTimer.get() < 3.0) {
			return SystemState.INTAKING;
		}

		switch (m_wantedState) {
			case CLIMB_EXTEND:
				return SystemState.CLIMB_EXTEND;
			case CLIMB_RETRACT:
				return SystemState.CLIMB_RETRACT;
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
		m_kicker.set(true);
		m_leds.setOn(true);
		m_leds.setColor(Color.YELLOW, 1.0f);

		if (!Constants.USE_UNCOUNTED_SHOOTING && m_stateChanged && m_inventory.getFilledChamberCount() > 0) {
			final var loadingChamber = m_inventory.getOptimalShootingChamber();
			final var currentChamber = m_inventory.getNearestChamber(Inventory.Location.SHOOTER);
			m_hopper.rotate(currentChamber.getDistance(loadingChamber));
		}

		if (m_periodicIO.target.isPresent()) {
			final var target = m_periodicIO.target.get();
			final var goal = target.getOptimalGoal();
			final var outerDistance = target.getFieldToOuterGoal().distance(m_periodicIO.turretPose.getTranslation());

			m_hood.setDeployed(outerDistance > Constants.CLOSE_SHOT_RANGE_METERS);
			setTurretTargetPoint(goal);

			if (isAligned(target, true) && (Constants.USE_UNCOUNTED_SHOOTING || m_inventory.getFilledChamberCount() > 0)) {
				return SystemState.WAITING_FOR_FLYWHEEL;
			}
		} else {
			setTurretFieldRotation(Rotation.degrees(0));
		}

		switch (m_wantedState) {
			case CLIMB_EXTEND:
				return SystemState.CLIMB_EXTEND;
			case CLIMB_RETRACT:
				return SystemState.CLIMB_RETRACT;
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
		m_leds.setColor(Color.ORANGE, 1.0f);

		final var target = m_periodicIO.target.get();
		final var targetDistance = target.getOptimalGoal().distance(m_periodicIO.turretPose.getTranslation());

		setTurretTargetPoint(target.getOptimalGoal());

		final double targetRPM;

		if (targetDistance < Constants.WALL_SHOOTING_RPM) {
			targetRPM = Constants.WALL_SHOOTING_RPM;
		} else {
			targetRPM = Constants.getRPM(targetDistance);
		}

		m_shooter.setRPM(targetRPM);

		if (!isAligned(m_periodicIO.target.get(), true)) {
			return SystemState.WAITING_FOR_ALIGNMENT;
		}

		final var turretAtRef = m_turret.atReference();
		final var readyToFire = isReadyToFire();
		final var shooterAtRef = m_shooter.atReference();
		final var hoodAtRef = m_hood.wantsDeployed() == m_hood.isDeployed();

		SmartDashboard.putBoolean("Turret At Ref", turretAtRef);
		SmartDashboard.putBoolean("Shooter At Ref", shooterAtRef);
		SmartDashboard.putBoolean("Hood At Ref", hoodAtRef);
		SmartDashboard.putBoolean("Ready To Fire", readyToFire);

		if (turretAtRef && readyToFire) {
			return SystemState.SHOOTING;
		}

		switch (m_wantedState) {
			case CLIMB_EXTEND:
				return SystemState.CLIMB_EXTEND;
			case CLIMB_RETRACT:
				return SystemState.CLIMB_RETRACT;
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
		m_leds.setColor(Color.RED, 1.0f);

		setTurretTargetPoint(m_periodicIO.target.get().getOptimalGoal());

		if (m_stateChanged) {
			m_hopper.rotate(6);
		}

		// wait for the entire motion
		if (!m_hopper.atRest()) {
			return SystemState.SHOOTING;
		}

		// make sure the last ball gets out
		if (m_wantedState != WantedState.SHOOTING && m_wantStateChangeTimer.get() < STOP_SHOOT_TIME_SECONDS) {
			return SystemState.SHOOTING;
		}

		switch (m_wantedState) {
			case CLIMB_EXTEND:
				return SystemState.CLIMB_EXTEND;
			case CLIMB_RETRACT:
				return SystemState.CLIMB_RETRACT;
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

	private boolean isAligned(final RobotState.CachedTarget target, final boolean allowDeadspot) {
		final var wheelSpeeds = m_chassis.getWheelProperty(MotorController::getVelocityLinearMetersPerSecond).map(Math::abs);
		final var outerDistance = target.getFieldToOuterGoal().distance(m_periodicIO.turretPose.getTranslation());
		final var isCloseShot = outerDistance < Constants.CLOSE_SHOT_RANGE_METERS;
		final var isFarShot = allowDeadspot && outerDistance > (Constants.CLOSE_SHOT_RANGE_METERS + Units.feet2Meters(1.5));

		return m_turret.atReference()
			   && (wheelSpeeds.left + wheelSpeeds.right) < 0.04
			   && ((isCloseShot && target.getDriveDistanceSinceCapture() < TARGET_CACHE_DISTANCE) || (isFarShot && target.getAge() < 0.25));
	}

	private boolean isReadyToFire() {
		return m_shooter.atReference()
			   && m_hopper.atRest()
			   && m_hood.isDeployed() == m_hood.wantsDeployed();
	}

	public void setTurretTargetPoint(final Translation targetPoint) {
		m_turret.setReferenceRotation(targetPoint
									  .difference(m_periodicIO.turretPose.getTranslation())
									  .direction()
									  .difference(m_periodicIO.vehiclePose.getRotation()));
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
