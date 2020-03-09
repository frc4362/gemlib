package com.gemsrobotics.frc2020.subsystems;

import com.gemsrobotics.frc2020.Constants;
import com.gemsrobotics.frc2020.Target;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.subsystems.Limelight;
import com.gemsrobotics.lib.utils.Units;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.annotations.Log;

import java.awt.*;
import java.util.Objects;
import java.util.Optional;

import static com.gemsrobotics.lib.utils.MathUtils.Tau;
import static com.gemsrobotics.lib.utils.MathUtils.epsilonEquals;
import static java.lang.Math.PI;
import static java.lang.Math.abs;

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
		WAITING_FOR_HOPPER,
		SHOOTING,
		CLIMB_EXTEND,
		CLIMB_RETRACT,
		WAITING_FOR_KICKER
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
	private final Inventory m_inventory;

	private MotorController<CANSparkMax> m_intakeMotor;
	private Solenoid m_intakeDeployer, m_kicker;
	private DoubleSolenoid m_pto;

	private final Timer m_stateChangeTimer, m_wantStateChangeTimer, m_intakeTimer, m_intakeStallTimer;

	private final PeriodicIO m_periodicIO;

	@Log.ToString
	private SystemState m_systemState;
	@Log.ToString
	private WantedState m_wantedState;
	private boolean m_stateChanged;
	private DigitalInput m_intakeSensor;

	private Rotation m_turretGuess;
	private int m_shotsFired;

	private Superstructure() {
		m_leds = LEDs.getInstance();
		m_targetServer = TargetServer.getInstance();
		m_chassis = Chassis.getInstance();
		m_shooter = Shooter.getInstance();
		m_turret = Turret.getInstance();
		m_hood = Hood.getInstance();
		m_hopper = Hopper.getInstance();
		m_targetState = RobotState.getInstance();
		m_inventory = new Inventory();

		m_intakeMotor = MotorControllerFactory.createDefaultSparkMax(Constants.CHANNEL_LEFT_PORT);
		m_intakeMotor.setInvertedOutput(true);

		m_intakeSensor = new DigitalInput(7);
		m_kicker = new Solenoid(Constants.KICKER_SOLENOID_PORT);
		m_intakeDeployer = new Solenoid(Constants.INTAKE_SOLENOID_PORT);
		m_pto = new DoubleSolenoid(Constants.PTO_SOLENOID_PORT[0], Constants.PTO_SOLENOID_PORT[1]);

		m_intakeTimer = new Timer();
		m_stateChangeTimer = new Timer();
		m_wantStateChangeTimer = new Timer();
		m_intakeStallTimer = new Timer();

		m_systemState = SystemState.IDLE;

		m_turretGuess = Rotation.identity();
		m_shotsFired = 0;

		m_periodicIO = new PeriodicIO();
	}

	private static class PeriodicIO {
		private RigidTransform vehiclePose = RigidTransform.identity();
		private RigidTransform turretPose = RigidTransform.identity();
		private Optional<Target> target = Optional.empty();
		private boolean oldIntakeSensor = false;
		private boolean intakeSensor = false;
	}

	public synchronized void setTurretGuess(final Rotation turretGuess) {
		m_turretGuess = turretGuess;
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

	public synchronized int getShotsFired() {
		return m_shotsFired;
	}

	@Override
	protected synchronized void readPeriodicInputs(final double timestamp) {
		m_periodicIO.vehiclePose = m_targetState.getFieldToVehicle(timestamp);
		m_periodicIO.turretPose = m_targetState.getFieldToTurret(timestamp);
		m_periodicIO.target = (Constants.USE_SCUFFED_WALLSHOT && DriverStation.getInstance().getStickButton(2, 7)) ? Optional.of(new WallshotTarget(m_periodicIO.vehiclePose.getTranslation())): m_targetState.getCachedFieldToTarget();
		m_periodicIO.oldIntakeSensor = m_periodicIO.intakeSensor;
		m_periodicIO.intakeSensor = !m_intakeSensor.get();
	}

	private static class WallshotTarget implements Target {
		private Translation m_t;

		public WallshotTarget(final Translation outer) {
			m_t = outer;
		}

		@Override
		public double getAge() {
			return 0.0;
		}

		@Override
		public boolean isClose() {
			return true;
		}

		@Override
		public double getDriveDistanceSinceCapture() {
			return 0.0;
		}

		@Override
		public Translation getFieldToOuterGoal() {
			return m_t.getTranslation().translateBy(new Translation(1.0, 0.0));
		}

		public Optional<Translation> getFieldToInnerGoal() {
			return Optional.empty();
		}
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

		SmartDashboard.putNumber("Intake Current", m_intakeMotor.getDrawnCurrent());
		SmartDashboard.putString("Wanted State", m_wantedState.toString());
		SmartDashboard.putString("Current State", m_systemState.toString());

		final var facingTarget = abs(m_periodicIO.turretPose.getRotation().getDegrees()) < 75.0;
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
			case WAITING_FOR_HOPPER:
				newState = handleWaitingForHopper();
				break;
			case WAITING_FOR_KICKER:
				newState = handleWaitingForKicker();
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
		m_hood.setDeployed(true);
		m_shooter.setDisabled();
		m_intakeDeployer.set(false);
		m_intakeMotor.setNeutral();

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
		m_hood.setDeployed(true);
		m_hopper.setDisabled();
		m_shooter.setDisabled();
		m_intakeDeployer.set(false);
		m_intakeMotor.setNeutral();

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
		m_hood.setDeployed(true);
		m_hopper.setDisabled();
		m_shooter.setDisabled();
		m_intakeDeployer.set(false);
		m_intakeMotor.setNeutral();

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
		m_hood.setDeployed(true);
		m_shooter.setDisabled();

		m_intakeDeployer.set(true);

		if (m_hopper.atRest() && m_stateChangeTimer.get() > 0.1 && m_wantedState == WantedState.INTAKING) {
			m_inventory.setRotations(m_hopper.getRotations());

			if (!m_periodicIO.intakeSensor && m_periodicIO.oldIntakeSensor && m_stateChangeTimer.get() > 0.25) {
				m_intakeTimer.start();
			} else if (m_intakeTimer.get() > 1.0) {
				m_intakeTimer.stop();
				m_intakeTimer.reset();
				m_hopper.rotate(-1);
				m_inventory.getNearestChamber(Inventory.Location.LEFT_INTAKE).setFull(true);
			} else if (m_intakeTimer.get() > 0.25){
				m_intakeMotor.setNeutral();
			} else {
				m_intakeMotor.setDutyCycle(1.0);
			}
		} else {
			m_intakeMotor.setNeutral();
		}

		if (m_wantedState != WantedState.INTAKING && m_wantStateChangeTimer.get() < 0.1) {
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

	private SystemState handleOuttaking() {
		m_turret.setDisabled();
		m_hood.setDeployed(true);
		m_shooter.setDisabled();

		m_intakeDeployer.set(true);

		if (m_wantedState == WantedState.OUTTAKING && m_wantStateChangeTimer.get() < 0.1) {
			m_intakeMotor.setDutyCycle(0.0);
		} else if (m_wantedState == WantedState.OUTTAKING && m_wantStateChangeTimer.get() > 0.1) {
			m_intakeMotor.setDutyCycle(-0.5);
		} else if (m_wantedState != WantedState.OUTTAKING && m_wantStateChangeTimer.get() > 0.1) {
			m_intakeMotor.setDutyCycle(0.0);
			return SystemState.OUTTAKING;
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

	private SystemState handleWaitingForAlignment() {
		m_leds.setOn(true);
		m_leds.setColor(Color.YELLOW, 1.0f);
		m_intakeDeployer.set(false);
		m_intakeMotor.setNeutral();

		if (Constants.USE_MANAGED_INVENTORY && m_stateChanged && m_inventory.getFilledChamberCount() > 0) {
			final var loadingChamber = m_inventory.getOptimalShootingChamber();
			final var currentChamber = m_inventory.getNearestChamber(Inventory.Location.SHOOTER);
			m_hopper.rotate(currentChamber.getDistance(loadingChamber));
		}

		if (m_periodicIO.target.isPresent()) {
			final var target = m_periodicIO.target.get();
			final var goal = target.getOptimalGoal();
			final var outerDistance = target.getFieldToOuterGoal().distance(m_periodicIO.turretPose.getTranslation());

			m_hood.setDeployed(!target.isClose());

			if (target.isClose()) {
				m_turret.setReferenceRotation(Rotation.degrees(0));
			} else {
				setTurretTargetGoal(goal);
			}

			if (isAligned(target, true) && (!Constants.USE_MANAGED_INVENTORY || m_inventory.getFilledChamberCount() > 0)) {
				return SystemState.WAITING_FOR_FLYWHEEL;
			}
		} else {
			m_turret.setReferenceRotation(m_turretGuess);
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
		m_leds.setColor(Color.MAGENTA, 1.0f);

		final var target = m_periodicIO.target.get();
		final var targetDistance = target.getOptimalGoal().distance(m_periodicIO.turretPose.getTranslation());

		if (target.isClose()) {
			m_turret.setReferenceRotation(Rotation.degrees(0));
		} else {
			setTurretTargetGoal(target.getOptimalGoal());
		}

		final double targetRPM;

		if (target.isClose()) {
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

		if (turretAtRef && readyToFire && m_hopper.atRest() && epsilonEquals(m_targetServer.getOffsetHorizontal().getDegrees(), 0.0, 1.0)) {
			return SystemState.WAITING_FOR_HOPPER;
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

	private SystemState handleWaitingForHopper() {
		m_leds.setColor(Color.PINK, 1.0f);

		if (m_stateChanged) {
			m_hopper.rotate(2);
		}

		if (m_hopper.atRest() && m_stateChangeTimer.get() > 0.1) {
			return SystemState.WAITING_FOR_KICKER;
		} else {
			return SystemState.WAITING_FOR_HOPPER;
		}
	}

	private SystemState handleWaitingForKicker() {
		if (m_stateChanged) {
			m_kicker.set(true);
		}

		if (m_stateChangeTimer.get() > 0.1) {
			return SystemState.SHOOTING;
		} else {
			return SystemState.WAITING_FOR_KICKER;
		}
	}

	private SystemState handleShooting() {
		m_leds.setColor(Color.RED, 1.0f);

		final Target target;

		if (m_periodicIO.target.isPresent()) {
			target = m_periodicIO.target.get();
		} else {
			return SystemState.IDLE;
		}

		if (target.isClose()) {
			m_turret.setReferenceRotation(Rotation.degrees(0));
		} else {
			setTurretTargetGoal(target.getOptimalGoal());
		}

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

		if (m_wantedState == WantedState.SHOOTING) {
			return SystemState.SHOOTING;
		} else {
			return SystemState.IDLE;
		}
	}

	@Override
	public void setSafeState() {

	}

	private boolean isAligned(final Target target, final boolean allowDeadspot) {
		final var wheelSpeeds = m_chassis.getWheelProperty(MotorController::getVelocityLinearMetersPerSecond).map(Math::abs);
		final var outerDistance = target.getFieldToOuterGoal().distance(m_periodicIO.turretPose.getTranslation());

		return m_turret.atReference()
			   && (wheelSpeeds.left + wheelSpeeds.right) < 0.04
			   && ((target.isClose() && target.getDriveDistanceSinceCapture() < TARGET_CACHE_DISTANCE) || (!target.isClose() && target.getAge() < 0.25));
	}

	private boolean isReadyToFire() {
		return m_shooter.atReference()
			   && m_hopper.atRest()
			   && m_hood.isDeployed() == m_hood.wantsDeployed();
	}

	private void setTurretTargetGoal(final Translation hint) {
		if (m_targetServer.isTargetPresent()) {
			if (abs(m_targetServer.getOffsetHorizontal().getDegrees()) > 0.5) {
				m_turret.setPositionOffset(m_targetServer.getOffsetHorizontal());
			} else {
				m_turret.setDisabled();
			}
		} else {
			setTurretFieldRotation(hint.difference(m_periodicIO.turretPose.getTranslation()).direction());
		}
	}

	private void setTurretFieldRotation(final Rotation fieldRotation) {
		m_turret.setReferenceRotation(fieldRotation.difference(m_periodicIO.vehiclePose.getRotation()));
	}
}
