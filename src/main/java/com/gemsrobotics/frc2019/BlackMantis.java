package com.gemsrobotics.frc2019;

import com.gemsrobotics.lib.structure.SingleThreadedSubsystemManager;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import java.util.List;

import static com.gemsrobotics.lib.utils.MathUtils.powSign;
import static java.lang.Math.abs;

public final class BlackMantis extends TimedRobot {
	private Chassis m_chassis;
	private SingleThreadedSubsystemManager m_manager;
	private XboxController m_gamepad;
	private CANSparkMax m_spark4;
	private CANSparkMax m_spark1;
	private CANSparkMax m_spark2;
	private CANSparkMax m_spark3;
	private edu.wpi.first.wpilibj.drive.DifferentialDrive drive;
	private PowerDistributionPanel m_pdp;

	@Override
	public void robotInit() {
		m_chassis = Chassis.getInstance();
		m_manager = new SingleThreadedSubsystemManager(List.of(m_chassis));
		m_gamepad = new XboxController(0);
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

		m_chassis.setCurvatureDrive(powSign(leftY, 2.0), rightX, m_gamepad.getRightBumper());

		m_manager.update();
	}
}
