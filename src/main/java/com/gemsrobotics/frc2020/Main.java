package com.gemsrobotics.frc2020;

import com.gemsrobotics.frc2020.subsystems.Shooter;
import edu.wpi.first.wpilibj.RobotBase;

public class Main {
	public static void main(final String[] args) {
		RobotBase.startRobot(Overload::new);
	}
}
