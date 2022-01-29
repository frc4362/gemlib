package com.gemsrobotics.frc2020.subsystems;

import com.gemsrobotics.frc2020.Constants;
import com.gemsrobotics.frc2020.Target;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.subsystems.Limelight;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.annotations.Log;

import java.awt.*;
import java.util.Objects;
import java.util.Optional;

import static com.gemsrobotics.lib.utils.MathUtils.epsilonEquals;
import static java.lang.Math.abs;

public final class Superstructure extends Subsystem {
	public static final double TARGET_CACHE_DISTANCE = 5.0;
	public static final double STOP_SHOOT_TIME_SECONDS = 0.6;
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
		CLIMB_EXTEND,
		CLIMB_RETRACT
	}

	public enum SystemState {
		IDLE,
		INTAKING,
		OUTTAKING,
		WAITING_FOR_ALIGNMENT,
		WAITING_FOR_FLYWHEEL,
		WAITING_FOR_HOPPER,
		WAITING_FOR_KICKER,
		SHOOTING,
		CLIMB_EXTEND,
		CLIMB_RETRACT
	}

	private final LEDs m_leds;
	private final TargetServer m_targetServer;
	private final Chassis m_chassis;
	private final Shooter m_shooter;
	private final ArmabotTurret240 m_turret;
	private final Hood m_hood;
	private final Spindexer m_spindexer;
	private final RobotState m_robotState;

//	private MotorController<CANSparkMax> m_intakeMotorLower, m_intakeMotorUpper;
	private Solenoid m_intakeDeployer, m_kicker;
	private DoubleSolenoid m_pto;

	private final Timer m_stateChangeTimer, m_wantStateChangeTimer;

	private final PeriodicIO m_periodicIO;

	@Log.ToString
	private SystemState m_systemState;
	@Log.ToString
	private WantedState m_wantedState;
	private boolean m_stateChanged;

	private Rotation m_turretGuess;

	private Superstructure() {
		m_leds = LEDs.getInstance();
		m_targetServer = TargetServer.getInstance();
		m_chassis = Chassis.getInstance();
		m_shooter = Shooter.getInstance();
		m_turret = ArmabotTurret240.getInstance();
		m_hood = Hood.getInstance();
		m_spindexer = Spindexer.getInstance();
		m_robotState = RobotState.getInstance();

		m_kicker = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.KICKER_SOLENOID_PORT);
		m_intakeDeployer = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.INTAKE_SOLENOID_PORT);
		m_pto = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.PTO_SOLENOID_PORT[0], Constants.PTO_SOLENOID_PORT[1]);

		m_stateChangeTimer = new Timer();
		m_wantStateChangeTimer = new Timer();

		m_turretGuess = Rotation.identity();

		m_systemState = SystemState.IDLE;
		m_periodicIO = new PeriodicIO();
	}

	private static class PeriodicIO {
		private RigidTransform vehiclePose = RigidTransform.identity();
		private RigidTransform turretPose = RigidTransform.identity();
		private Optional<Target> target = Optional.empty();
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

	private SystemState applyWantedState() {
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
	protected synchronized void readPeriodicInputs(final double timestamp) {
		m_periodicIO.vehiclePose = m_robotState.getFieldToVehicle(timestamp);
		m_periodicIO.turretPose = m_robotState.getFieldToTurret(timestamp);
		m_periodicIO.target = (Constants.USE_SCUFFED_WALLSHOT && DriverStation.getStickButton(2, 7))
						  ? Optional.of(new WallshotTarget(m_periodicIO.vehiclePose.getTranslation()))
						  : m_robotState.getCachedFieldToTarget();
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
		m_turret.setDisabled();
		m_hood.setDeployed(true);
		m_shooter.setDisabled();
		m_intakeDeployer.set(false);

		m_leds.setColor(Color.BLACK, 0.0f);
		m_pto.set(DoubleSolenoid.Value.kReverse);

		return applyWantedState();
	}

	private SystemState handleClimbExtend() {
		m_turret.setDisabled();
		m_hood.setDeployed(true);
		m_spindexer.setDisabled();
		m_shooter.setDisabled();
		m_intakeDeployer.set(false);

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
		m_spindexer.setDisabled();
		m_shooter.setDisabled();
		m_intakeDeployer.set(false);

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

		if (m_stateChangeTimer.get() > 0.1) {
//			m_intakeMotorLower.setDutyCycle(1.0);
//			m_intakeMotorUpper.setDutyCycle(1.0);
		}

		if (m_wantedState != WantedState.INTAKING && m_wantStateChangeTimer.get() < 0.1) {
			return SystemState.INTAKING;
		}

		return applyWantedState();
	}

	private SystemState handleOuttaking() {
		m_turret.setDisabled();
		m_hood.setDeployed(true);
		m_shooter.setDisabled();

		m_intakeDeployer.set(true);

		if (m_wantedState == WantedState.OUTTAKING && m_wantStateChangeTimer.get() < 0.1) {
		} else if (m_wantedState == WantedState.OUTTAKING && m_wantStateChangeTimer.get() > 0.1) {
		} else if (m_wantedState != WantedState.OUTTAKING && m_wantStateChangeTimer.get() > 0.1) {
			return SystemState.OUTTAKING;
		}

		return applyWantedState();
	}

	// does not turn off the shooter; you can drop of out of WAITING_FOR_FLYWHEEL and recover
	private SystemState handleWaitingForAlignment() {
		m_leds.setOn(true);
		m_leds.setColor(Color.BLUE, 1.0f);
		m_intakeDeployer.set(false);

		if (m_periodicIO.target.isPresent()) {
			final var target = m_periodicIO.target.get();
			final var goal = target.getOptimalGoal();
			final var outerDistance = target.getFieldToOuterGoal().distance(m_periodicIO.turretPose.getTranslation());

			m_hood.setDeployed(!DriverStation.getStickButton(1, 4));

			if (target.isClose()) {
				m_turret.setReference(Rotation.degrees(0));
			} else {
				setTurretTargetGoal(goal);
			}

			if (isAligned(target, true)) {
				System.out.println("Alignment found, go to WAITING_FOR_FLYWHEEL");
				return SystemState.WAITING_FOR_FLYWHEEL;
			}
		} else {
			m_turret.setReference(m_turretGuess);
		}

		return applyWantedState();
	}

	private SystemState handleWaitingForFlywheel() {
		m_leds.setColor(Color.MAGENTA, 1.0f);

		final var target = m_periodicIO.target.get();
		final var targetDistance = target.getOptimalGoal().distance(m_periodicIO.turretPose.getTranslation());

		if (target.isClose()) {
			m_turret.setReference(Rotation.degrees(0));
		} else {
			setTurretTargetGoal(target.getOptimalGoal());
		}

		final double targetRPM;

		if (target.isClose()) {
			targetRPM = Constants.WALL_SHOOTING_RPM;
		} else {
			targetRPM = Constants.getRPM(targetDistance);
		}

//		m_shooter.setRPM(SmartDashboard.getNumber("Shooter RPM", 0));
		m_shooter.setRPM(targetRPM);

		if (!isAligned(m_periodicIO.target.get(), true)) {
			System.out.println("Target lost, dropping back to WAITING_FOR_ALIGNMENT");
			return SystemState.WAITING_FOR_ALIGNMENT;
		}

		final var turretAtRef = m_turret.atReference();
		final var readyToFire = isReadyToFire();
		final var shooterAtRef = m_shooter.atReference();
		final var hoodAtRef = m_hood.wantsDeployed() == m_hood.isDeployed();

		SmartDashboard.putBoolean("Turret At Ref", turretAtRef);
		SmartDashboard.putBoolean("Shooter At Ref", shooterAtRef);
		SmartDashboard.putBoolean("Hood At Ref", hoodAtRef);
//		SmartDashboard.putBoolean("Ready To Fire", readyToFire);

		if (turretAtRef && readyToFire && epsilonEquals(m_targetServer.getOffsetHorizontal().getDegrees(), 0.0, 1.0)) {
			System.out.println("Stopping the hopper");
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
		m_leds.setColor(Color.GREEN, 1.0f);

		if (m_stateChanged) {
			m_spindexer.setShootingPosition();
		}

		if (m_spindexer.atRest()) {
			System.out.println("Hopper stopped, go to KICKER state");
			return SystemState.WAITING_FOR_KICKER;
		} else {
			return SystemState.WAITING_FOR_HOPPER;
		}
	}

	private SystemState handleWaitingForKicker() {
		if (m_stateChanged) {
			m_kicker.set(true);
		}

		m_spindexer.setDisabled();

		if (m_stateChangeTimer.get() > 0.25) {
			System.out.println("Kicker engaged, go to SHOOTING state");
			return SystemState.SHOOTING;
		} else {
			return SystemState.WAITING_FOR_KICKER;
		}
	}

	private SystemState handleShooting() {
		m_leds.setColor(Color.RED, 1.0f);

		if (m_stateChanged) {
			System.out.println("Entered SHOOTING state, turning spindexer");
			m_spindexer.rotate(8);
		}

		if (m_wantedState != WantedState.SHOOTING && m_stateChangeTimer.get() > 3.0) {
			System.out.println("3 seconds have passed, going to IDLE state");
			return SystemState.IDLE;
		}

		final Target target;

		if (m_periodicIO.target.isPresent()) {
			target = m_periodicIO.target.get();
		} else {
			System.out.println("Target not found in SHOOTING state. How did you get here? Going to IDLE state");
			return SystemState.IDLE;
		}

		if (target.isClose()) {
			m_turret.setReference(Rotation.degrees(0));
		} else {
			setTurretTargetGoal(target.getOptimalGoal());
		}

		// wait for the entire motion
		if (!m_spindexer.atRest()) {
			return SystemState.SHOOTING;
		}

		if (m_wantedState != WantedState.SHOOTING && m_wantStateChangeTimer.get() < STOP_SHOOT_TIME_SECONDS) {
			System.out.println("Wants to transition out, keep shooting a bit");
			return SystemState.SHOOTING;
		} else if (m_wantedState == WantedState.SHOOTING) {
			return SystemState.SHOOTING;
		} else {
			System.out.println("Shoot sequence complete, go to IDLE state");
			return SystemState.IDLE;
		}
	}

	@Override
	public void setSafeState() {

	}

	private boolean isAligned(final Target target, final boolean allowDeadspot) {
		final var wheelSpeeds = m_chassis.getWheelProperty(MotorController::getVelocityLinearMetersPerSecond).map(Math::abs);
		final var outerDistance = target.getFieldToOuterGoal().distance(m_periodicIO.turretPose.getTranslation());

		final var a = m_turret.atReference();
		final var b = (wheelSpeeds.left + wheelSpeeds.right) < 0.04;
		final var c = ((target.isClose() && target.getDriveDistanceSinceCapture() < TARGET_CACHE_DISTANCE) || (!target.isClose() && target.getAge() < 0.25));

		return a && b && c;
	}

	private boolean isReadyToFire() {
		return m_shooter.atReference() && (m_hood.isDeployed() == m_hood.wantsDeployed());
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
		m_turret.setReference(fieldRotation.difference(m_periodicIO.vehiclePose.getRotation()));
	}

	public Solenoid getKicker() {
		return m_kicker;
	}
}
