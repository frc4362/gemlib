package com.gemsrobotics.frc2020;

import com.gemsrobotics.frc2020.subsystems.*;
import com.gemsrobotics.frc2020.subsystems.RobotState;
import com.gemsrobotics.lib.commands.TrackTrajectoryCommand;
import com.gemsrobotics.lib.drivers.hid.Gemstick;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.RigidTransformWithCurvature;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.structure.SubsystemManager;
import com.gemsrobotics.lib.subsystems.Limelight;
import com.gemsrobotics.lib.subsystems.drivetrain.WheelState;
import com.gemsrobotics.lib.trajectory.TrajectoryContainer;
import com.gemsrobotics.lib.utils.MathUtils;
import com.gemsrobotics.lib.utils.Units;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.Loggable;

import java.util.Arrays;
import java.util.Collections;

public final class Overload extends TimedRobot implements Loggable {
	private Chassis m_chassis;
	private Hopper m_hopper;
	private Shooter m_shooter;
	private Turret m_turret;
	private TargetServer m_targetServer;
	private RobotState m_robotState;
	private Superstructure m_superstructure;

	private SubsystemManager m_subsystemManager;
	private MotorController<CANSparkMax> m_1, m_2, m_3;
	private Solenoid m_intakeSol;

	Compressor m_compressor;
	Gemstick m_stickLeft, m_stickRight;
	XboxController m_gamepad;

	TrajectoryContainer<RigidTransformWithCurvature> m_trajectory;

	boolean m_slowDrive;

	@Override
	public void robotInit() {
		m_chassis = Chassis.getInstance();
		m_stickLeft = new Gemstick(0);
		m_stickRight = new Gemstick(1, Gemstick.Deadbands.makeRectangleDeadband(0.06, 0.15));
		m_shooter = Shooter.getInstance();
		m_hopper = Hopper.getInstance();
		m_turret = Turret.getInstance();
		m_targetServer = TargetServer.getInstance();
		m_robotState = RobotState.getInstance();
		m_compressor = new Compressor();
		m_compressor.setClosedLoopControl(false);
		m_superstructure = Superstructure.getInstance();

		m_intakeSol = new Solenoid(Constants.INTAKE_SOLENOID_PORT);

		m_1 = MotorControllerFactory.createDefaultSparkMax(Constants.CHANNEL_RIGHT_PORT);
		m_1.setInvertedOutput(false);
		m_2 = MotorControllerFactory.createDefaultSparkMax(Constants.CHANNEL_CENTER_PORT);
		m_2.setInvertedOutput(true);
		m_3 = MotorControllerFactory.createDefaultSparkMax(Constants.CHANNEL_LEFT_PORT);
		m_3.setInvertedOutput(true);

		m_subsystemManager = new SubsystemManager(m_targetServer, m_hopper, m_chassis, m_turret, m_robotState, m_superstructure, m_shooter);
		m_gamepad = new XboxController(2);

//		m_h = MotorControllerFactory.createSparkMax(40, MotorControllerFactory.DEFAULT_SPARK_CONFIG);

		SmartDashboard.putNumber("Shooter RPM", 0.0);
		SmartDashboard.putNumber("Drive Train Speed", 0.0);
		SmartDashboard.putNumber("Turret Degrees", 0.0);
		SmartDashboard.putNumber("Intake Speed", 0.0);
		SmartDashboard.putNumber("Hopper Voltage", 0.0);

		m_targetServer.setLEDMode(Limelight.LEDMode.OFF);

		m_trajectory = m_chassis.getTrajectoryGenerator().generateTrajectory(
				false,
				false,
				Arrays.asList(RigidTransform.identity(), new RigidTransform(Units.feet2Meters(10.0), 0.0, Rotation.identity())),
				Collections.emptyList());

//		Logger.configureLogging(this);
	}

	@Override
	public void robotPeriodic() {
//		Logger.updateEntries();
	}

	@Override
	public void disabledInit() {
		m_targetServer.setLEDMode(Limelight.LEDMode.OFF);
//		m_superstructure.setWantedState(Superstructure.WantedState.IDLE);
	}

	@Override
	public void autonomousInit() {
		m_subsystemManager.start();
		Scheduler.getInstance().add(new TrackTrajectoryCommand(m_chassis, m_trajectory));
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	boolean intakeOut;

	@Override
	public void teleopInit() {
		Scheduler.getInstance().removeAll();
		intakeOut = false;
		m_superstructure.setWantedState(Superstructure.WantedState.IDLE);
		m_subsystemManager.start();
		m_targetServer.setLEDMode(Limelight.LEDMode.ON);
		m_slowDrive = false;
	}

	@Override
	public void teleopPeriodic() {
		SmartDashboard.putString("Heading", m_chassis.getHeading().toString());

		SmartDashboard.putString("Robot Pose", m_robotState.getLatestFieldToVehicle().toString());
		final var target = m_robotState.getCachedFieldToTarget();
		SmartDashboard.putString("Target Pose", target.map(RobotState.CachedTarget::getFieldToOuterGoal).map(Translation::toString).orElse("None"));

		SmartDashboard.putString("Horizontal Offset", m_targetServer.getOffsetHorizontal().toString());

		final var wheelSpeeds = m_chassis.getWheelProperty(MotorController::getVelocityLinearMetersPerSecond);
		SmartDashboard.putNumber("Linear Velocity", (wheelSpeeds.left + wheelSpeeds.right) / 2.0);

		final var wheelDraws = m_chassis.getWheelProperty(MotorController::getDrawnCurrent);
		SmartDashboard.putNumber("Left Current Draw", wheelDraws.left);
		SmartDashboard.putNumber("Right Current Draw", wheelDraws.right);

//		m_turret.setReferenceRotation(Rotation.degrees(SmartDashboard.getNumber("Turret Rotation", 0.0)));

		m_intakeSol.set(intakeOut);

		if (m_gamepad.getAButtonPressed()) {
			intakeOut = !intakeOut;
		} else if (m_gamepad.getBButtonPressed()) {
			m_hopper.rotate(1);
		} else if (m_gamepad.getXButtonPressed()) {
			m_hopper.rotate(6);
		}

		final double intakeSpeed;

		if (intakeOut) {
			intakeSpeed = m_gamepad.getTriggerAxis(GenericHID.Hand.kRight) * 0.5;
			SmartDashboard.putNumber("Intake %", intakeSpeed);
		} else {
			intakeSpeed = 0.0;
		}

		m_1.setDutyCycle(intakeSpeed);
		m_2.setDutyCycle(intakeSpeed);
		m_3.setDutyCycle(intakeSpeed);

		SmartDashboard.putNumber("Right Throttle", -m_stickRight.getY());

		if (m_superstructure.getSystemState() == Superstructure.SystemState.CLIMB_EXTEND) {
			m_chassis.setOpenLoop(new WheelState(-m_stickLeft.getY() * 0.2, -m_stickRight.getY() * 0.2));

			if (m_stickRight.getRawButtonPressed(12)) {
				m_superstructure.setWantedState(Superstructure.WantedState.IDLE);
				m_slowDrive = true;
			}
		} else if (m_superstructure.getSystemState() == Superstructure.SystemState.CLIMB_RETRACT) {
			m_chassis.setOpenLoop(new WheelState(MathUtils.powSign(-m_stickLeft.getY(), 2), MathUtils.powSign(-m_stickRight.getY(), 2)));

			if (m_stickRight.getRawButtonPressed(10)) {
				m_superstructure.setWantedState(Superstructure.WantedState.IDLE);
				m_slowDrive = false;
			}
		} else {
			double throttle = -m_stickLeft.getY();
			double wheel = -m_stickRight.getX();
			boolean quickturn = m_stickRight.getRawButton(3);
			final double multiplier = m_slowDrive ? 0.25 : 1.0;

			m_chassis.setCurvatureDrive(Math.copySign(throttle * throttle, throttle) * multiplier, wheel, quickturn);

			if (m_stickRight.getTrigger()) {
				m_superstructure.setWantedState(Superstructure.WantedState.SHOOTING);
			} else if (m_stickRight.getRawButtonPressed(11)) {
				m_superstructure.setWantedState(Superstructure.WantedState.CLIMB_EXTEND);
			} else if (m_stickRight.getRawButtonPressed(9)) {
				m_superstructure.setWantedState(Superstructure.WantedState.CLIMB_RETRACT);
			} else {
				m_superstructure.setWantedState(Superstructure.WantedState.IDLE);
			}
		}
	}
}
