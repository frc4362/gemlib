package com.gemsrobotics.frc2019.util;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANPIDController;

public class PIDF {
	public double kP, kI, kD, kF, kIZ;

	public void configure(final CANPIDController controller) {
		controller.setP(kP);
		controller.setI(kI);
		controller.setD(kD);
		controller.setFF(kF);
		controller.setIZone(kIZ);
	}

	public void configure(final WPI_TalonSRX controller) {
		controller.config_kP(0, kP);
		controller.config_kI(0, kI);
		controller.config_kD(0, kD);
		controller.config_kF(0, kF);
		controller.config_IntegralZone(0, (int) kIZ);
	}
}
