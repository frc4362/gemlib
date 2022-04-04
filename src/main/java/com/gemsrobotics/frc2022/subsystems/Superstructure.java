package com.gemsrobotics.frc2022.subsystems;

import com.gemsrobotics.frc2022.Constants;
import com.gemsrobotics.frc2022.ShotParameters;
import com.gemsrobotics.frc2022.subsystems.uptake.Uptake;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.subsystems.Flywheel;
import com.gemsrobotics.lib.subsystems.Limelight;
import com.gemsrobotics.lib.utils.FastDoubleToString;
import com.gemsrobotics.lib.utils.MathUtils;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Objects;
import java.util.Optional;

import static java.lang.Math.abs;

public final class Superstructure extends Subsystem {
	private static final double SHOOTING_ALLOWED_SPEED = 0.1;
	private static final double SHOOTER_REJECT_VELOCITY = 4.0;
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
		LOW_SHOT,
		SHOOTING,
		SHOOTING_AND_INTAKING,
		PRECLIMBING,
		CLIMBING
	}

	public enum SystemState {
		IDLE,
		INTAKING,
		OUTTAKING,
		LOW_SHOT,
		WAITING_FOR_FLYWHEEL,
		SHOOTING,
		REJECTING_CARGO,
		PRECLIMB,
		PULL_TO_BAR,
		PLACE_ON_BAR,
		EXTEND_TO_BAR,
		GRAB_BAR,
		DONE
	}

	public enum ClimbGoal {
		HIGH(1),
		TRAVERSE(2);

		public final int allowedClimbs;

		ClimbGoal(final int a) {
			allowedClimbs = a;
		}
	}

	private final Chassis m_chassis;
	private final Intake m_intake;
	private final Uptake m_uptake;
	private final Flywheel m_shooterLower, m_shooterUpper;
	private final Climber m_climber;
	private final FieldState m_fieldState;
	private final GreyTTurret m_turret;
	private final Timer m_stateChangedTimer, m_wantStateChangeTimer;
	private final PeriodicIO m_periodicIO;
	private final DataLog m_log;
	private final StringLogEntry m_shotLogger;

	private Optional<Rotation> m_turretGuess;
	private int m_climbCount;
	private double m_shotAdjustment;
	private boolean m_stateChanged, m_prepareShot, m_turretLocked, m_weakShot;
	private Optional<ClimbGoal> m_climbGoal;
	private WantedState m_stateWanted;
	private SystemState m_state;

	private Superstructure() {
		m_intake = Intake.getInstance();
		m_uptake = Uptake.getInstance();
		m_shooterLower = LowerWheel.getInstance();
		m_shooterUpper = UpperWheel.getInstance();
		m_climber = Climber.getInstance();
		m_chassis = Chassis.getInstance();
		m_fieldState = FieldState.getInstance();
		m_turret = GreyTTurret.getInstance();
		m_turretGuess = Optional.empty();

		m_stateChangedTimer = new Timer();
		m_wantStateChangeTimer = new Timer();
		m_periodicIO = new PeriodicIO();

		m_log = DataLogManager.getLog();
		m_shotLogger = new StringLogEntry(m_log, "/shots");

		m_prepareShot = false;
		m_weakShot = false;
		m_turretLocked = false;

		m_shotAdjustment = 0;
		m_climbCount = 0;
		m_climbGoal = Optional.empty();
		m_state = SystemState.IDLE;
		m_stateWanted = WantedState.IDLE;
	}

	private static class PeriodicIO {
		public RigidTransform vehiclePose = RigidTransform.identity();
		public Rotation turretPose = Rotation.identity();
		public Optional<ShotParameters> shotParameters = Optional.empty();
	}

	public synchronized void setWantedState(final WantedState newState) {
		if (newState != m_stateWanted) {
			m_stateWanted = newState;
			m_wantStateChangeTimer.reset();
		}
	}

	@Override
	protected void readPeriodicInputs(double timestamp) {
		m_periodicIO.turretPose = m_turret.getEstimatedRotation(timestamp);
		m_periodicIO.vehiclePose = m_fieldState.getFieldToVehicle(timestamp);
		m_periodicIO.shotParameters = m_fieldState.getCachedFieldToTarget();
	}

	@Override
	protected void onStart(final double timestamp) {
		m_wantStateChangeTimer.start();
		m_stateChangedTimer.start();
	}

	@Override
	protected void onUpdate(final double timestamp) {
		if (m_state == SystemState.PRECLIMB
			|| m_state == SystemState.PULL_TO_BAR
			|| m_state == SystemState.PLACE_ON_BAR
			|| m_state == SystemState.GRAB_BAR
			|| m_state == SystemState.DONE
			|| m_state == SystemState.EXTEND_TO_BAR
			|| m_state == SystemState.LOW_SHOT
//			|| DriverStation.getStickButton(0, 6)
		) {
			m_turret.setReference(Rotation.identity());
		} else if (m_turretLocked) {
			m_turret.setDisabled();
		} else if (m_periodicIO.shotParameters.isPresent()) {
			setTurretFieldRotation(m_periodicIO.shotParameters.get().getCurrentTurretToGoal().direction());
		} else if (m_turretGuess.isPresent()) {
			m_turret.setReference(m_turretGuess.get());
		} else {
			m_turret.setDisabled();
		}

		final SystemState newState;

		SmartDashboard.putString("Wanted State", m_stateWanted.toString());
		SmartDashboard.putString("Current State", m_state.toString());
		SmartDashboard.putNumber("Shot Adjustment", m_shotAdjustment);

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
			case WAITING_FOR_FLYWHEEL:
				newState = handleWaitingForFlywheel();
				break;
			case LOW_SHOT:
			case SHOOTING:
			case REJECTING_CARGO:
				newState = handleShooting();
				break;
			case PRECLIMB:
				newState = handlePreclimb();
				break;
			case PULL_TO_BAR:
				newState = handlePullToBar();
				break;
			case PLACE_ON_BAR:
				newState = handlePlaceOnBar();
				break;
			case EXTEND_TO_BAR:
				newState = handleExtendToBar();
				break;
			case GRAB_BAR:
				newState = handleGrabBar();
				break;
			case DONE:
				newState = handleDone();
				break;
			default:
				newState = SystemState.IDLE;
				break;
		}

		if (newState != m_state) {
			m_state = newState;
			m_stateChangedTimer.reset();
			m_stateChanged = true;
		} else {
			m_stateChanged = false;
		}
	}

	@Override
	protected void onStop(final double timestamp) {
		m_wantStateChangeTimer.stop();
		m_wantStateChangeTimer.reset();
		m_stateChangedTimer.stop();
		m_stateChangedTimer.reset();
	}

	private SystemState applyWantedState() {
		switch (m_stateWanted) {
			case INTAKING:
				return SystemState.INTAKING;
			case OUTTAKING:
				return SystemState.OUTTAKING;
			case LOW_SHOT:
			case SHOOTING:
			case SHOOTING_AND_INTAKING:
				return SystemState.WAITING_FOR_FLYWHEEL;
			case PRECLIMBING:
				return SystemState.PRECLIMB;
			case CLIMBING:
				return SystemState.PULL_TO_BAR;
			case IDLE:
			default:
				return SystemState.IDLE;
		}
	}

	private SystemState handleIdle() {
		m_intake.setWantedState(Intake.State.RETRACTED);
		m_uptake.setWantedState(Uptake.State.NEUTRAL);
		setShooterDeferring();

		return applyWantedState();
	}

	private SystemState handleIntaking() {
		m_intake.setWantedState(Intake.State.INTAKING);
		m_uptake.setWantedState(Uptake.State.INTAKING);
		setShooterDeferring();

		if (Constants.DO_CARGO_REJECT && m_uptake.isWrongCargoHeld()) {
			return SystemState.REJECTING_CARGO;
		} else {
			return applyWantedState();
		}
	}

	private SystemState handleOuttaking() {
		m_intake.setWantedState(Intake.State.OUTTAKING);
		m_uptake.setWantedState(Uptake.State.OUTTAKING);
		setShooterDeferring();

		return applyWantedState();
	}

	private SystemState handleWaitingForFlywheel() {
//		setShooterLinearVelocity(DriverStation.getStickButton(0, B button number) ? 2.5 : getVisionVelocity());
		setShooterLinearVelocity(getIntendedVelocity());

		m_intake.setWantedState(m_stateWanted == WantedState.SHOOTING_AND_INTAKING ? Intake.State.INTAKING : Intake.State.RETRACTED);
		m_uptake.setWantedState(m_stateWanted == WantedState.SHOOTING_AND_INTAKING ? Uptake.State.INTAKING : Uptake.State.NEUTRAL);

		final var chassisSpeeds = m_chassis.getWheelProperty(MotorController::getVelocityLinearMetersPerSecond);
		final var leftOk = abs(chassisSpeeds.left) < SHOOTING_ALLOWED_SPEED;
		final var rightOk = abs(chassisSpeeds.right) < SHOOTING_ALLOWED_SPEED;
		final var rangeOk = true;// getVisionDistance().map(distance -> distance > 1.7 && distance < 2.85).orElse(true);

		if (m_shooterUpper.atReference()
			&& m_shooterLower.atReference()
//			&& m_turret.atReference()
			&& (DriverStation.isTeleop() || leftOk)
			&& (DriverStation.isTeleop() || rightOk)
			&& (DriverStation.isAutonomous() || (m_turret.getVelocityPerSecond().getDegrees()) < 40.0)
			&& (DriverStation.isAutonomous() || rangeOk)
		) {
			return SystemState.SHOOTING;
		} else if (m_stateWanted == WantedState.SHOOTING) {
			return SystemState.WAITING_FOR_FLYWHEEL;
		} else {
			return applyWantedState();
		}
	}

	private SystemState handleShooting() {
		final var intendedVelocity = getIntendedVelocity();
		setShooterLinearVelocity(intendedVelocity);

		if (Constants.DO_SHOOTER_LOGGING && m_stateChanged) {
			m_shotLogger.append(getVisionDistance().map(FastDoubleToString::format).orElse("nil") + ": " + FastDoubleToString.format(intendedVelocity));
		}

		var wantsIntake = m_stateWanted == WantedState.SHOOTING_AND_INTAKING || m_stateWanted == WantedState.INTAKING;
		m_intake.setWantedState(wantsIntake ? Intake.State.INTAKING : Intake.State.RETRACTED);
		var revTimeRejectPassed = m_stateChangedTimer.get() > 0.05;
		m_uptake.setWantedState((m_state != SystemState.REJECTING_CARGO || revTimeRejectPassed) ? Uptake.State.FEEDING : Uptake.State.INTAKING);

		if (m_state == SystemState.REJECTING_CARGO && m_stateChangedTimer.get() < 0.4) {
			return SystemState.REJECTING_CARGO;
		} else if (m_stateChangedTimer.get() < 0.75 || m_stateWanted == WantedState.SHOOTING) {
			return SystemState.SHOOTING;
		} else {
			return applyWantedState();
		}
	}

	private SystemState handlePreclimb() {
		m_climber.setVoltageSettings(Climber.VoltageSetting.HIGH);
		m_climber.setPreclimbHeight();
		TargetServer.getInstance().setLEDMode(Limelight.LEDMode.OFF);

		if (m_stateWanted == WantedState.CLIMBING) {
			return SystemState.PULL_TO_BAR;
		} else {
			return SystemState.PRECLIMB;
		}
	}

	private SystemState handlePullToBar() {
		if (m_stateChanged) {
			m_climber.setVoltageSettings(Climber.VoltageSetting.LOW);
		}

		if (m_climbCount >= m_climbGoal.map(goal -> goal.allowedClimbs).orElse(0)) {
			return SystemState.DONE;
		}

		if (m_climbCount > 0 && m_stateChangedTimer.get() < 0.5) {
			return SystemState.PULL_TO_BAR;
		}

		m_climber.setReferencePercent(0.0);

		if (m_climber.atReference()) {
			return SystemState.PLACE_ON_BAR;
		} else {
			return SystemState.PULL_TO_BAR;
		}
	}

	private SystemState handlePlaceOnBar() {
		if (m_stateChanged) {
			m_climber.setSwingerExtended(true);
			m_climber.setVoltageSettings(Climber.VoltageSetting.LOW);
		}

		final double waitDuration = 1.0;

//		once the solenoid has actuated, reach for the high bar
		if (m_stateChangedTimer.get() > waitDuration) {
			m_climber.setReferencePercent(0.08);
		} else {
			return SystemState.PLACE_ON_BAR;
		}

		if (m_climber.atReference()) {
			return SystemState.EXTEND_TO_BAR;
		}

		return SystemState.PLACE_ON_BAR;
	}

	private SystemState handleExtendToBar() {
		if (m_stateChanged) {
			m_climber.setVoltageSettings(Climber.VoltageSetting.MID);
		}

		m_climber.setReferencePercent(0.72);

		if (m_chassis.getPitch().getDegrees() > 41.5 && m_climber.atReference()) {
			return SystemState.GRAB_BAR;
		} else {
			return SystemState.EXTEND_TO_BAR;
		}
	}

	private SystemState handleGrabBar() {
		m_climber.setReferencePercent(0.95);

		if (m_climber.atReference()) {
			m_climber.setSwingerExtended(false);
			m_climbCount++;
			return SystemState.PULL_TO_BAR;
		} else {
			return SystemState.GRAB_BAR;
		}
	}

	private SystemState handleDone() {
//		if (m_stateChangedTimer.get() < 5.0) {
//			return SystemState.DONE;
//		}

		m_climber.setReferencePercent(0.7);

		return SystemState.DONE;
	}

	private Optional<Double> getVisionDistance() {
		return m_periodicIO.shotParameters.map(shot -> shot.getCurrentTurretToGoal().norm());
	}

	private double getIntendedVelocity() {
//		return SmartDashboard.getNumber("Shooter Velocity Meters", 0.0);

		if (m_stateWanted == WantedState.LOW_SHOT) {
			return 4.0;
		}

		return m_periodicIO.shotParameters.map(shotParameters ->
			   Constants.getRPM(shotParameters.getCurrentTurretToGoal().norm())).orElse(0.0);
	}

	private void setTurretFieldRotation(final Rotation fieldRotation) {
		m_turret.setReference(fieldRotation.difference(m_periodicIO.vehiclePose.getRotation()));
	}

	private void setShooterLinearVelocity(double velocity) {
		// constrain to peak linear velocity of the top wheel
		velocity += m_shotAdjustment;
		velocity = MathUtils.coerce(0.0, velocity, 19.93);

		if (velocity <= 1) {
			m_shooterLower.setVelocityMetersPerSecond(0);
			m_shooterUpper.setVelocityMetersPerSecond(0);
		} else {
			m_shooterLower.setVelocityMetersPerSecond(velocity - 1.);
			m_shooterUpper.setVelocityMetersPerSecond(velocity - 1.);
		}
	}

	// use for preparing the shot asap
	public void setShooterDeferring() {
		if (Constants.DO_EARLY_FLYWHEEL && m_prepareShot) {
			setShooterLinearVelocity(getIntendedVelocity());
		} else {
			setShooterLinearVelocity(0.0);
		}
	}

	public void setTurretGuess(final Rotation guess) {
		m_turretGuess = Optional.of(guess);
	}

	public void setPrepareShot(final boolean prepareShot) {
		m_prepareShot = prepareShot;
	}

	public SystemState getSystemState() {
		return m_state;
	}

	public void adjustShot(final double adjustment) {
		m_shotAdjustment += adjustment;
	}

	public void setWantedStateClimb(final ClimbGoal goal) {
		setWantedState(WantedState.CLIMBING);
		m_climbGoal = Optional.of(goal);
	}

	public void setTurretLocked(final boolean locked) {
		m_turretLocked = locked;
	}

	@Override
	public void setSafeState() {

	}
}
