package com.gemsrobotics.frc2022.subsystems;

import com.gemsrobotics.frc2022.Constants;
import com.gemsrobotics.frc2022.FieldState;
import com.gemsrobotics.frc2022.ShotParameters;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.subsystems.Flywheel;
import com.gemsrobotics.lib.utils.MathUtils;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.awt.*;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Function;

public final class Superstructure extends Subsystem {
	private static final int ALLOWED_CLIMBS = 2;
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
		SHOOTING,
		PRECLIMBING,
		CLIMBING
	}

	public enum SystemState {
		IDLE,
		INTAKING,
		OUTTAKING,
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

	private final Chassis m_chassis;
	private final Intake m_intake;
	private final Uptake m_uptake;
	private final Flywheel m_shooterLower, m_shooterUpper;
	private final Climber m_climber;
	private final FieldState m_fieldState;
	private final GreyTTurret m_turret;

	private final Timer m_stateChangedTimer, m_wantStateChangeTimer;
	private final PeriodicIO m_periodicIO;
	private int m_climbCount;
	private boolean m_stateChanged;
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

		m_stateChangedTimer = new Timer();
		m_wantStateChangeTimer = new Timer();

		m_periodicIO = new PeriodicIO();

		m_climbCount = 0;
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
//		m_leds.setOn(true);
	}

	@Override
	protected void onUpdate(final double timestamp) {
//		m_leds.setColor(m_state.ledState.apply(timestamp), 1.0f);

		SmartDashboard.putString("Turret to Goal", m_periodicIO.shotParameters.map(ShotParameters::getCurrentTurretToGoal).map(Translation::toString).orElse("No target"));
		SmartDashboard.putString("Turret to Goal Direction", m_periodicIO.shotParameters.map(ShotParameters::getCurrentTurretToGoal).map(Translation::direction).map(Rotation::toString).orElse("None"));

		if (m_periodicIO.shotParameters.isPresent()) {
			setTurretFieldRotation(m_periodicIO.shotParameters.get().getCurrentTurretToGoal().direction());
		} else {
			m_turret.setDisabled();
		}

		final SystemState newState;

		SmartDashboard.putString("Wanted State", m_stateWanted.toString());
		SmartDashboard.putString("Current State", m_state.toString());

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
		setShooterLinearVelocity(0.0);

		return applyWantedState();
	}

	private SystemState handleIntaking() {
		m_intake.setWantedState(Intake.State.INTAKING);
		m_uptake.setWantedState(Uptake.State.INTAKING);
		setShooterLinearVelocity(0.0);

		return applyWantedState();
	}

	private SystemState handleOuttaking() {
		m_intake.setWantedState(Intake.State.OUTTAKING);
		m_uptake.setWantedState(Uptake.State.OUTTAKING);
		setShooterLinearVelocity(0.0);

		return applyWantedState();
	}

	private SystemState handleWaitingForFlywheel() {
		setShooterLinearVelocity(getVisionVelocity());

		m_intake.setWantedState(Intake.State.RETRACTED);

		if (m_shooterUpper.atReference() && m_shooterLower.atReference()) {
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

		if (m_stateChangedTimer.get() < 2.0 || m_stateWanted == WantedState.SHOOTING) {
			return SystemState.SHOOTING;
		} else {
			return applyWantedState();
		}
	}

	private SystemState handlePreclimb() {
		m_climber.setUseHighVoltage(true);
		m_climber.setPreclimbHeight();

		return applyWantedState();
	}

	private SystemState handleGrabMedBar() {
		if (m_stateChanged) {
			m_climber.setUseHighVoltage(false);
		}

		if (m_climbCount >= ALLOWED_CLIMBS) {
			return SystemState.DONE;
		}

		if (m_climbCount > 0 && m_stateChangedTimer.get() < 3.0) {
			return SystemState.GRAB_MED_BAR;
		}

		m_climber.setReferencePercent(0.0);

//		return SystemState.DONE;

		if (m_climber.atReference()) {
			return SystemState.EXTEND_HIGH_BAR;
		} else {
			return SystemState.GRAB_MED_BAR;
		}
	}

	private SystemState handleExtendHighBar() {
		if (m_stateChanged) {
			m_climber.setSwingerExtended(true);
//			m_intake.setWantedState(Intake.State.EXTENDED);
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
//			m_intake.setWantedState(Intake.State.RETRACTED);
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

	private double getVisionVelocity() {
		return m_periodicIO.shotParameters.map(shotParameters -> Constants.getRPM(shotParameters.getCurrentTurretToGoal().norm())).orElse(2.0);
	}

	private void setTurretFieldRotation(final Rotation fieldRotation) {
		m_turret.setReference(fieldRotation.difference(m_periodicIO.vehiclePose.getRotation()));
	}

	private void setShooterLinearVelocity(double velocity) {
		// constrain to peak linear velocity of the top wheel
		velocity = MathUtils.coerce(0.0, velocity, 19.93);

		m_shooterLower.setVelocityMetersPerSecond(velocity);
		m_shooterUpper.setVelocityMetersPerSecond(velocity);
	}

	public SystemState getSystemState() {
		return m_state;
	}

	@Override
	public void setSafeState() {

	}
}
