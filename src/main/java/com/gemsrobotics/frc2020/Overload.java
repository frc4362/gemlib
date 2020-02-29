package com.gemsrobotics.frc2020;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.frc2020.subsystems.*;
import com.gemsrobotics.lib.drivers.hid.Gemstick;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.structure.SubsystemManager;
import com.gemsrobotics.lib.subsystems.Limelight;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;

import java.util.List;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

public final class Overload extends TimedRobot implements Loggable {
	private Chassis m_chassis;
	private Hopper m_hopper;
	private Shooter m_shooter;
	private Turret m_turret;

	private Limelight m_limelight;
	private DoubleSolenoid m_pto;
	private Solenoid m_kicker, m_hood, m_intakeSol;
	private SubsystemManager m_subsystemManager;
	private MotorController<CANSparkMax> m_1, m_2, m_3, m_h;

	Compressor m_compressor;
	Gemstick m_stickLeft, m_stickRight;
	XboxController m_gamepad;

	@Override
	public void robotInit() {
		m_chassis = Chassis.getInstance();
		m_stickLeft = new Gemstick(0);
		m_stickRight = new Gemstick(1);
		m_shooter = Shooter.getInstance();
		m_hopper = Hopper.getInstance();
		m_turret = Turret.getInstance();
		m_compressor = new Compressor();
		m_compressor.setClosedLoopControl(false);

		m_limelight = new Limelight() {
			@Override
			protected void onStart(double timestamp) {
				setLEDMode(LEDMode.OFF);
			}

			@Override
			protected void onUpdate(double timestamp) {

			}

			@Override
			protected void onStop(double timestamp) {

			}
		};

		m_pto = new DoubleSolenoid(3, 4);

		m_hood = new Solenoid(Constants.HOOD_SOLENOID_PORT);
		m_kicker = new Solenoid(Constants.KICKER_SOLENOID_PORT);
		m_intakeSol = new Solenoid(Constants.INTAKE_SOLENOID_PORT);

		m_1 = MotorControllerFactory.createDefaultSparkMax(Constants.CHANNEL_RIGHT_PORT);
		m_1.setInvertedOutput(false);
		m_2 = MotorControllerFactory.createDefaultSparkMax(Constants.CHANNEL_CENTER_PORT);
		m_2.setInvertedOutput(true);
		m_3 = MotorControllerFactory.createDefaultSparkMax(Constants.CHANNEL_LEFT_PORT);
		m_3.setInvertedOutput(true);
		m_h = MotorControllerFactory.createDefaultSparkMax(Constants.HOPPER_PORT);

		m_subsystemManager = new SubsystemManager(m_chassis, m_hopper, m_turret, m_shooter);
		m_gamepad = new XboxController(2);

		SmartDashboard.putNumber("Shooter RPM", 0.0);
		SmartDashboard.putNumber("Drive Train Speed", 0.0);
		SmartDashboard.putNumber("Turret Degrees", 0.0);
		SmartDashboard.putNumber("Intake Speed", 0.0);

		Logger.configureLogging(this);
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void teleopInit() {
		new Compressor().setClosedLoopControl(false);
		m_subsystemManager.start();
	}

	@Override
	public void teleopPeriodic() {
		m_intakeSol.set(m_gamepad.getAButton());

		double throttle = -m_stickLeft.getY();
		double wheel = -m_stickRight.getX();
		boolean quickturn = m_stickRight.getRawButton(3);
		SmartDashboard.putNumber("throttle", throttle);
		SmartDashboard.putNumber("wheel", wheel);
//		m_chassis.setCurvatureDrive(Math.copySign(throttle * throttle, throttle), wheel, quickturn);

		//m_pto.set(DoubleSolenoid.Value.kForward);

		m_turret.setReferenceRotation(Rotation.degrees(SmartDashboard.getNumber("Turret Degrees", 0.0)));
		SmartDashboard.putNumber("Turret Encoder Pos", m_turret.m_motor.getInternalController().getSelectedSensorPosition());

		final double shooterRpm = SmartDashboard.getNumber("Shooter RPM", 0.0);
		m_shooter.setRPM(shooterRpm);

		final double intakeSpeed = m_gamepad.getTriggerAxis(GenericHID.Hand.kRight) * 0.5;
		m_1.setDutyCycle(intakeSpeed);
		m_2.setDutyCycle(intakeSpeed);
		m_3.setDutyCycle(intakeSpeed);

		if (m_gamepad.getXButtonPressed()) {
			m_hopper.rotate(1);
		} else if (m_gamepad.getBButtonPressed()) {
			m_hopper.rotate(6);
		} else if(m_gamepad.getYButtonPressed()){
			m_kicker.set(true);
		} else if(m_gamepad.getAButtonPressed()){
			m_kicker.set(false);
		}
	}
}
