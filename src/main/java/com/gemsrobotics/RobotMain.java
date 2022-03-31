package com.gemsrobotics;

import com.gemsrobotics.frc2022.Blackbird;
import edu.wpi.first.wpilibj.RobotBase;

public class RobotMain {
	public static void main(final String[] args) {
		RobotBase.startRobot(Blackbird::new);
	}
}
