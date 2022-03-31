package com.gemsrobotics.frc2020;

import com.gemsrobotics.frc2020.autonomous.*;
import com.gemsrobotics.frc2020.subsystems.*;
import com.gemsrobotics.frc2020.subsystems.RobotState;
import com.gemsrobotics.lib.commands.CharacterizeDifferentialDrive;
import com.gemsrobotics.lib.commands.WaitCommand;
import com.gemsrobotics.lib.drivers.hid.Gemstick;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.structure.SubsystemManager;
import com.gemsrobotics.lib.subsystems.Limelight;
import com.gemsrobotics.lib.subsystems.drivetrain.WheelState;
import com.gemsrobotics.lib.utils.MathUtils;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;

public final class Wallace extends TimedRobot implements Loggable {
	private Chassis m_chassis;
	private Spindexer m_spindexer;
	private Shooter m_shooter;
	private ArmabotTurret240 m_turret;
	private TargetServer m_targetServer;
	private RobotState m_robotState;
	private Hood m_hood;
	private Superstructure m_superstructure;

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
		m_spindexer = Spindexer.getInstance();
		m_turret = ArmabotTurret240.getInstance();
		m_targetServer = TargetServer.getInstance();
		m_robotState = RobotState.getInstance();
		m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
		m_superstructure = Superstructure.getInstance();

		m_compressorToggler = new SendableChooser<>();
		m_compressorToggler.setDefaultOption("Compressor OFF", false);
		m_compressorToggler.addOption("Compressor ON", true);

		m_subsystemManager = new SubsystemManager(m_targetServer, m_spindexer, m_chassis, m_turret, m_robotState, m_superstructure, m_shooter, m_hood);
		m_gamepad = new XboxController(2);

		m_autonChooser = new SendableChooser<>();
		m_autonChooser.setDefaultOption("None", new WaitCommand(1.0));
		m_autonChooser.addOption("Characterize Differential Drive", new CharacterizeDifferentialDrive(m_chassis, false));
		m_autonChooser.addOption("Bounce", new Bounce());

//		SmartDashboard.putData(m_compressorToggler);
//		SmartDashboard.putData(m_autonChooser);
//		SmartDashboard.putData(m_turret);

		m_chassis.getOdometer().reset(Timer.getFPGATimestamp(), RigidTransform.fromRotation(Rotation.degrees(0)));
		m_chassis.setHeading(Rotation.degrees(0));

		m_targetServer.setLEDMode(Limelight.LEDMode.OFF);

		Logger.setCycleWarningsEnabled(false);
		Logger.configureLoggingAndConfig(m_subsystemManager, false);
	}

	@Override
	public void robotPeriodic() {
		SmartDashboard.putNumber("Turret Absolute Encoder Position", m_turret.getAbsolutePosition());
		final var v = m_chassis.getWheelProperty(MotorController::getVelocityLinearMetersPerSecond);
		SmartDashboard.putNumber("Robot Velocity Left", v.left);
		SmartDashboard.putNumber("Robot Velocity Right", v.right);
		SmartDashboard.putString("Wheel Positions", m_chassis.getWheelProperty(MotorController::getPositionMeters).toString());
		SmartDashboard.putString("Robot Position", m_robotState.getLatestFieldToVehicle().toString());
//		Logger.updateEntries();
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
		m_compressor.disable();
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

		if (m_compressorToggler.getSelected()) {
			m_compressor.enableDigital();
		} else {
			m_compressor.disable();
		}
	}

	@Override
	public void teleopPeriodic() {
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
			} else if (m_gamepad.getRightBumper()) {
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
