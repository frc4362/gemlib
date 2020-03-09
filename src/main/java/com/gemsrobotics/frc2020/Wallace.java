package com.gemsrobotics.frc2020;

import com.gemsrobotics.frc2020.autonomous.SiuxBallAuton;
import com.gemsrobotics.frc2020.autonomous.ThreeBallAuton;
import com.gemsrobotics.frc2020.commands.DriveStraightCommand;
import com.gemsrobotics.frc2020.subsystems.*;
import com.gemsrobotics.frc2020.subsystems.RobotState;
import com.gemsrobotics.lib.commands.TrackTrajectoryCommand;
import com.gemsrobotics.lib.commands.WaitCommand;
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
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.Loggable;

import java.util.Arrays;
import java.util.Collections;

public final class Wallace extends TimedRobot implements Loggable {
	private Chassis m_chassis;
	private Hopper m_hopper;
	private Shooter m_shooter;
	private Turret m_turret;
	private TargetServer m_targetServer;
	private RobotState m_robotState;
	private Hood m_hood;
	private Superstructure m_superstructure;

	private Solenoid m_kicker;

	private SubsystemManager m_subsystemManager;

	private Compressor m_compressor;
	private Gemstick m_stickLeft, m_stickRight;
	private XboxController m_gamepad;

	private SendableChooser<Boolean> m_compressorToggler;
	private SendableChooser<Command> m_autonChooser;

	private boolean m_slowDrive;

	@Override
	public void robotInit() {
		m_hood = Hood.getInstance();
		m_chassis = Chassis.getInstance();
		m_stickLeft = new Gemstick(0);
		m_stickRight = new Gemstick(1, Gemstick.Deadbands.makeRectangleDeadband(0.06, 0.15));
		m_shooter = Shooter.getInstance();
		m_hopper = Hopper.getInstance();
		m_turret = Turret.getInstance();
		m_targetServer = TargetServer.getInstance();
		m_robotState = RobotState.getInstance();
		m_compressor = new Compressor();
		m_superstructure = Superstructure.getInstance();

//		m_kicker = new Solenoid(Constants.KICKER_SOLENOID_PORT);

		m_compressorToggler = new SendableChooser<>();
		m_compressorToggler.setDefaultOption("Compressor OFF", false);
		m_compressorToggler.addOption("Compressor ON", true);

		m_subsystemManager = new SubsystemManager(m_targetServer, m_hopper, m_chassis, m_turret, m_robotState, m_superstructure, m_shooter, m_hood);
		m_gamepad = new XboxController(2);

		SmartDashboard.putNumber("Shooter RPM", 0.0);

		m_autonChooser = new SendableChooser<>();
		m_autonChooser.setDefaultOption("None", new WaitCommand(1.0));
		m_autonChooser.addOption("3 Ball Auton (Seek Left)", new ThreeBallAuton(Rotation.degrees(160)));
		m_autonChooser.addOption("3 Ball Auton (Seek Right)", new ThreeBallAuton(Rotation.degrees(-160)));
		m_autonChooser.addOption("6 Ball Auton (Seek Right)", new SiuxBallAuton(Rotation.degrees(-160)));
		m_autonChooser.addOption("Drive Straight Test", new DriveStraightCommand(m_chassis, 0.15, Units.feet2Meters(3.0)));

		SmartDashboard.putData(m_compressorToggler);
		SmartDashboard.putData(m_autonChooser);

		m_chassis.getOdometer().reset(Timer.getFPGATimestamp(), RigidTransform.fromRotation(Rotation.degrees(180)));
		m_chassis.setHeading(Rotation.degrees(180));

		m_targetServer.setLEDMode(Limelight.LEDMode.OFF);
	}

	@Override
	public void robotPeriodic() {
		SmartDashboard.putNumber("Turret Absolute Encoder Position", m_turret.getAbsolutePosition());
	}

	@Override
	public void disabledInit() {
		m_subsystemManager.stop();
		m_targetServer.setLEDMode(Limelight.LEDMode.OFF);
	}

	@Override
	public void autonomousInit() {
		m_subsystemManager.start();
		Scheduler.getInstance().add(m_autonChooser.getSelected());
		m_compressor.setClosedLoopControl(false);
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		Scheduler.getInstance().removeAll();
		m_subsystemManager.start();
		m_superstructure.setWantedState(Superstructure.WantedState.IDLE);
		m_slowDrive = false;
		m_compressor.setClosedLoopControl(m_compressorToggler.getSelected());
	}

	int h, k;

	@Override
	public void teleopPeriodic() {
		SmartDashboard.putString("Heading", m_chassis.getHeading().toString());

		SmartDashboard.putString("Camera to Target", m_targetServer.getTargetInfo().map(TargetServer.TargetInfo::getCameraToTarget).map(RigidTransform::toString).orElse("None"));

		SmartDashboard.putString("Robot Pose", m_robotState.getLatestFieldToVehicle().toString());
		final var target = m_robotState.getCachedFieldToTarget();
//		SmartDashboard.putString("Target Pose", target.map(RobotState.CachedTarget::getFieldToOuterGoal).map(Translation::toString).orElse("None"));

		SmartDashboard.putString("Horizontal Offset", m_targetServer.getOffsetHorizontal().toString());

		if (m_gamepad.getBButtonPressed()) {
			m_hopper.rotate(-1);
		} else if (m_gamepad.getXButtonPressed()) {
			m_hopper.rotate(-6);
		} else if (m_gamepad.getYButtonPressed()) {
			m_hopper.assertSafe();
		}

		SmartDashboard.putNumber("Right Throttle", -m_stickRight.getY());

//		if (m_gamepad.getAButtonPressed()) {
//			k += 1;
//		} else if (m_gamepad.getYButtonPressed()) {
//			h += 1;
//		}

//		m_kicker.set(k % 2 == 1);
//		m_hood.setDeployed(h % 2 == 0);
//		m_shooter.setRPM(SmartDashboard.getNumber("Shooter RPM", 0.0));

		if (m_superstructure.getSystemState() == Superstructure.SystemState.CLIMB_EXTEND) {
			m_chassis.setOpenLoop(new WheelState(-m_stickLeft.getY(), -m_stickRight.getY()));

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

			if (m_stickRight.getTriggerPressed()) {
				m_superstructure.setWantedState(Superstructure.WantedState.SHOOTING);
			} else if (m_stickRight.getRawButtonPressed(11)) {
				m_superstructure.setWantedState(Superstructure.WantedState.CLIMB_EXTEND);
			} else if (m_stickRight.getRawButtonPressed(9)) {
				m_superstructure.setWantedState(Superstructure.WantedState.CLIMB_RETRACT);
			} else if (m_gamepad.getBumper(GenericHID.Hand.kRight)) {
				m_superstructure.setWantedState(Superstructure.WantedState.OUTTAKING);
			} else if (m_stickRight.getRawButton(2)) {
				m_superstructure.setWantedState(Superstructure.WantedState.INTAKING);
			} else if (!m_stickRight.getTrigger()) {
				m_superstructure.setWantedState(Superstructure.WantedState.IDLE);
			}
		}

		if (m_stickRight.getRawButton(10)) {
			m_slowDrive = false;
		}
	}
}
