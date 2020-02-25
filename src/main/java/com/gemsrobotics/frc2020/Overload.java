package com.gemsrobotics.frc2020;

import com.gemsrobotics.frc2020.subsystems.Chassis;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.structure.SubsystemManager;
import com.gemsrobotics.lib.subsystems.Limelight;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

public final class Overload extends TimedRobot {
	private Chassis m_chassis;
	private Limelight m_limelight;
	private DoubleSolenoid m_pto;
	private Solenoid m_c, m_d, m_e;
	private SubsystemManager m_subsystemManager;
	private MotorController<CANSparkMax> m_1, m_2, m_3;

	@Override
	public void robotInit() {
		m_chassis = Chassis.getInstance();
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
		m_c = new Solenoid(2);
		m_d = new Solenoid(1);
		m_e = new Solenoid(0);

		m_1 = MotorControllerFactory.createDefaultSparkMax(Constants.CHANNEL_RIGHT_PORT);
		m_2 = MotorControllerFactory.createDefaultSparkMax(Constants.CHANNEL_CENTER_PORT);
		m_3 = MotorControllerFactory.createDefaultSparkMax(Constants.CHANNEL_LEFT_PORT);

		m_subsystemManager = new SubsystemManager(m_chassis);
	}

	Timer t = new Timer();

	@Override
	public void disabledInit() {
		m_pto.set(DoubleSolenoid.Value.kReverse);
	}

	@Override
	public void teleopInit() {
		m_subsystemManager.start();
		t.start();
	}

	@Override
	public void teleopPeriodic() {
		m_chassis.setCurvatureDrive(0.0, 0.0, false);

		m_pto.set(DoubleSolenoid.Value.kForward);
	}
}
