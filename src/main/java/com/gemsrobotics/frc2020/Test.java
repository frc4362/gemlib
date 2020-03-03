package com.gemsrobotics.frc2020;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.TimedRobot;

public class Test extends TimedRobot {
	private CANSparkMax m_motor;

	@Override
	public void robotInit() {
		m_motor = new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless);
	}

	@Override
	public void teleopPeriodic() {
		m_motor.set(0.35);
	}
}
