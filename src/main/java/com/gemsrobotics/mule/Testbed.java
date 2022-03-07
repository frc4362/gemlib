package com.gemsrobotics.mule;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.structure.SingleThreadedSubsystemManager;
import edu.wpi.first.wpilibj.*;

import static java.lang.Math.abs;

public final class Testbed extends TimedRobot {
	private Chassis m_chassis;
	private SingleThreadedSubsystemManager m_manager;
	private XboxController m_gamepad;
	private TalonFX m_talon;
	private edu.wpi.first.wpilibj.drive.DifferentialDrive drive;
	private PowerDistributionPanel m_pdp;
	private DoubleSolenoid m_sole;
	private PneumaticHub m_ph;
	private Compressor m_compressor;

	@Override
	public void robotInit() {
//		m_chassis = Chassis.getInstance();
//		m_manager = new SingleThreadedSubsystemManager(List.of(m_chassis));
		m_gamepad = new XboxController(0);
//		m_talon = new TalonFX(0);
//		m_ph = new PneumaticHub(1);
//		m_sole = m_ph.makeDoubleSolenoid(8, 9);
//		m_compressor = m_ph.makeCompressor();
//		m_compressor.enableDigital();
	}

	@Override
	public void teleopInit() {
//		m_manager.start();
	}

	@Override
	public void teleopPeriodic() {
		double leftY = 0;
		double rightX = 0;

		//deadbanding
		if (abs(m_gamepad.getLeftY()) > 0.04) {
			leftY = -m_gamepad.getLeftY();
		}

		if (abs(m_gamepad.getRightX()) > 0.10) {
			rightX = m_gamepad.getRightX();
		}

//		m_sole.set(m_gamepad.getAButton() ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);

//		m_chassis.setCurvatureDrive(powSign(leftY, 2.0), rightX, m_gamepad.getRightBumper());

//		m_manager.update();
	}
}
