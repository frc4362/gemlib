package com.gemsrobotics.demo2022;

import com.gemsrobotics.demo2022.subsystems.Chassis;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.structure.SubsystemManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public final class Demobot extends TimedRobot {
	private Chassis m_chassis;
	private SubsystemManager m_subsystemManager;
	private XboxController m_gamepad;

	@Override
	public void robotInit() {
		m_gamepad = new XboxController(0);

		m_chassis = Chassis.getInstance();
		m_subsystemManager = new SubsystemManager(m_chassis);

		m_chassis.getOdometer().reset(Timer.getFPGATimestamp(), RigidTransform.identity());
		m_chassis.setHeading(Rotation.degrees(0));
	}

	@Override
	public void disabledInit() {
		m_subsystemManager.stop();
	}

	@Override
	public void teleopInit() {
		m_subsystemManager.start();
	}

	@Override
	public void teleopPeriodic() {
		double leftY = 0;
		double rightX = 0;
		//deadbanding 10%
		if(m_gamepad.getLeftY() > 0.1 || m_gamepad.getLeftY() < -0.1){
			leftY = m_gamepad.getLeftY();
		}
		if(m_gamepad.getRightX() > 0.1 || m_gamepad.getRightX() < -0.1){
			rightX = m_gamepad.getRightX();
		}

		m_chassis.setCurvatureDrive(leftY, rightX, m_gamepad.getRightBumper());
	}
}
