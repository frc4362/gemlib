package com.gemsrobotics;

import com.gemsrobotics.frc2019.BlackMantis;
import com.gemsrobotics.frc2022.Demobot;
import edu.wpi.first.wpilibj.RobotBase;

public class RobotMain {
	public static void main(final String[] args) {
		RobotBase.startRobot(Demobot::new);
	}
}
