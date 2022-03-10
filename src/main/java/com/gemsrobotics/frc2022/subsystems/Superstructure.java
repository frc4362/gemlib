package com.gemsrobotics.frc2022.subsystems;

import com.gemsrobotics.frc2022.Constants;
import com.gemsrobotics.frc2022.ShotParameters;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.subsystems.Flywheel;
import com.gemsrobotics.lib.subsystems.Limelight;
import com.gemsrobotics.lib.utils.MathUtils;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.awt.*;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Function;

import static java.lang.Math.abs;

public final class Superstructure extends Subsystem {
	private static final int ALLOWED_CLIMBS = 2;
	public static final double SHOOTING_ALLOWED_SPEED = 0.1;
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
		PRECLIMB(f -> Color.RED),
		GRAB_MED_BAR(f -> Color.BLUE),
		EXTEND_HIGH_BAR(f -> Color.GREEN),
		GRAB_HIGH_BAR(f -> Color.YELLOW),
		DONE(f -> Color.ORANGE);

		public final Function<Double, Color> ledState;

		SystemState(final Function<Double, Color> s) {
			ledState = s;
		}

		SystemState() {
			this(f -> Color.BLACK);
		}
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
	private Optional<Rotation> m_turretGuess;
	private int m_climbCount;
	private double m_shotAdjustment;
	private boolean m_stateChanged, m_prepareShot, m_turretLocked;
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

		m_prepareShot = false;
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
			|| m_state == SystemState.GRAB_MED_BAR
			|| m_state == SystemState.EXTEND_HIGH_BAR
			|| m_state == SystemState.GRAB_HIGH_BAR
			|| m_state == SystemState.DONE
			|| m_state == SystemState.LOW_SHOT
		) {
			m_turret.setReference(Rotation.identity());
		} else if (m_turretLocked) {
			m_turret.setDisabled();
		} else if (m_periodicIO.shotParameters.isPresent()) {
			setTurretFieldRotation(m_periodicIO.shotParameters.get().getCurrentTurretToGoal().direction());
//			m_turret.setDisabled();
		} else if (m_turretGuess.isPresent()) {
			m_turret.setReference(m_turretGuess.get());
		} else {
			m_turret.setDisabled();
		}

		final SystemState newState;

		SmartDashboard.putString("Wanted State", m_stateWanted.toString());
		SmartDashboard.putString("Current State", m_state.toString());
		SmartDashboard.putNumber("Shot Adjustment", m_shotAdjustment);
		SmartDashboard.putString("Shots", "root");

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
				newState = handleShooting();
				break;
			case PRECLIMB:
				newState = handlePreclimb();
				break;
			case GRAB_MED_BAR:
				newState = handleGrabMedBar();
				break;
			case EXTEND_HIGH_BAR:
				newState = handleExtendHighBar();
				break;
			case GRAB_HIGH_BAR:
				newState = handleGrabHighBar();
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
				return SystemState.WAITING_FOR_FLYWHEEL;
			case PRECLIMBING:
				return SystemState.PRECLIMB;
			case CLIMBING:
				return SystemState.GRAB_MED_BAR;
			case IDLE:
			default:
				return SystemState.IDLE;
		}
	}

	private SystemState handleIdle() {
		m_intake.setWantedState(Intake.State.RETRACTED);
		m_uptake.setWantedState(Uptake.State.NEUTRAL);
		setShooterDefer();

		return applyWantedState();
	}

	private SystemState handleIntaking() {
		m_intake.setWantedState(Intake.State.INTAKING);
		m_uptake.setWantedState(Uptake.State.INTAKING);
		setShooterDefer();

		return applyWantedState();
	}

	private SystemState handleOuttaking() {
		m_intake.setWantedState(Intake.State.OUTTAKING);
		m_uptake.setWantedState(Uptake.State.OUTTAKING);
		setShooterDefer();

		return applyWantedState();
	}

	private SystemState handleWaitingForFlywheel() {
//		setShooterLinearVelocity(DriverStation.getStickButton(0, B button number) ? 2.5 : getVisionVelocity());
		setShooterLinearVelocity(getVisionVelocity());

		m_intake.setWantedState(Intake.State.RETRACTED);

		final var chassisSpeeds = m_chassis.getWheelProperty(MotorController::getVelocityLinearMetersPerSecond);
		final var leftOk = abs(chassisSpeeds.left) < SHOOTING_ALLOWED_SPEED;
		final var rightOk = abs(chassisSpeeds.right) < SHOOTING_ALLOWED_SPEED;

		if (m_shooterUpper.atReference()
			&& m_shooterLower.atReference()
//			&& m_turret.atReference()
			&& (DriverStation.isTeleop() || leftOk)
			&& (DriverStation.isTeleop() || rightOk)
		) {
			return SystemState.SHOOTING;
		} else if (m_stateWanted == WantedState.SHOOTING) {
			return SystemState.WAITING_FOR_FLYWHEEL;
		} else {
			return applyWantedState();
		}
	}

	private SystemState handleShooting() {
		setShooterLinearVelocity(getVisionVelocity());

		m_uptake.setWantedState(Uptake.State.FEEDING);

		if (m_stateChangedTimer.get() < 0.75 || m_stateWanted == WantedState.SHOOTING) {
			return SystemState.SHOOTING;
		} else {
			return applyWantedState();
		}
	}

	private SystemState handlePreclimb() {
		m_climber.setUseHighVoltage(true);
		m_climber.setPreclimbHeight();
		TargetServer.getInstance().setLEDMode(Limelight.LEDMode.OFF);

		if (m_stateWanted == WantedState.CLIMBING) {
			return SystemState.GRAB_MED_BAR;
		} else {
			return SystemState.PRECLIMB;
		}
	}

	private SystemState handleGrabMedBar() {
		if (m_stateChanged) {
			m_climber.setUseHighVoltage(false);
		}

		if (m_climbCount >= m_climbGoal.map(goal -> goal.allowedClimbs).orElse(0)) {
			return SystemState.DONE;
		}

		if (m_climbCount > 0 && m_stateChangedTimer.get() < 3.0) {
			return SystemState.GRAB_MED_BAR;
		}

		m_climber.setReferencePercent(0.0);

		if (m_climber.atReference()) {
			return SystemState.EXTEND_HIGH_BAR;
		} else {
			return SystemState.GRAB_MED_BAR;
		}
	}

	private SystemState handleExtendHighBar() {
		if (m_stateChanged) {
			m_climber.setSwingerExtended(true);
		}

		final double waitDuration;

		if (m_climbCount == 1) {
			waitDuration = 2.5;
		} else {
			waitDuration = 1.0;
		}

//		once the solenoid has actuated, reach for the high bar
		if (m_stateChangedTimer.get() > waitDuration) {
			m_climber.setReferencePercent(0.75);
		} else {
			return SystemState.EXTEND_HIGH_BAR;
		}

		if (m_chassis.getPitch().getDegrees() < -37.5 && m_climber.atReference()) {
			return SystemState.GRAB_HIGH_BAR;
		}

		return SystemState.EXTEND_HIGH_BAR;
	}

	private SystemState handleGrabHighBar() {
		m_climber.setReferencePercent(0.95);

		if (m_climber.atReference()) {
			m_climber.setSwingerExtended(false);
			m_climbCount++;
			return SystemState.GRAB_MED_BAR;
		} else {
			return SystemState.GRAB_HIGH_BAR;
		}
	}

	private SystemState handleDone() {
		if (m_stateChangedTimer.get() < 2.0) {
			return SystemState.DONE;
		}

		m_climber.setReferencePercent(0.7);

		return SystemState.DONE;
	}

	private Double getVisionVelocity() {
		if (m_stateWanted == WantedState.LOW_SHOT) {
			return 2.5;
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

		m_shooterLower.setVelocityMetersPerSecond(velocity);
		m_shooterUpper.setVelocityMetersPerSecond(velocity);
	}

	// use for preparing the shot asap
	public void setShooterDefer() {
		if (Constants.DO_EARLY_FLYWHEEL && m_prepareShot) {
			setShooterLinearVelocity(getVisionVelocity());
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
