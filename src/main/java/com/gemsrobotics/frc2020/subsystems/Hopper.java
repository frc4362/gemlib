package com.gemsrobotics.frc2020.subsystems;

import com.gemsrobotics.frc2020.Constants;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.utils.Units;
import com.revrobotics.CANSparkMax;

import java.util.Objects;

public final class Hopper extends Subsystem {
	private static final MotorController.GearingParameters GEARING_PARAMETERS =
			new MotorController.GearingParameters(1.0 / 77.0, Units.inches2Meters(13.75) / 2.0, 1.0);

	private static Hopper INSTANCE;

	public static Hopper getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Hopper();
		}

		return INSTANCE;
	}

	private final MotorController<CANSparkMax> m_motor;

	private Hopper() {
		m_motor = MotorControllerFactory.createSparkMax(Constants.HOPPER_PORT, new MotorControllerFactory.SparkConfiguration());
		m_motor.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		m_motor.setGearingParameters(GEARING_PARAMETERS);
		m_motor.setSelectedProfile(0);
	}

	@Override
	protected void readPeriodicInputs(double timestamp) {

	}

	@Override
	protected void onCreate(double timestamp) {

	}

	@Override
	protected void onUpdate(double timestamp) {

	}

	@Override
	protected void onStop(double timestamp) {

	}

	@Override
	public void setSafeState() {

	}
}
