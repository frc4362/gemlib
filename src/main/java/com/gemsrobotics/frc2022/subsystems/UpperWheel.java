package com.gemsrobotics.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.controls.MotorFeedforward;
import com.gemsrobotics.lib.controls.PIDFController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerGroup;
import com.gemsrobotics.lib.subsystems.Flywheel;
import com.gemsrobotics.lib.utils.Units;

import java.util.List;
import java.util.Objects;

public final class UpperWheel extends Flywheel {
	private static final int SHOOTER_LOWER_WHEEL_PORT = 9;
	private static final double WHEEL_RADIUS_METERS = Units.inches2Meters(2.5 / 2.0);
	private static final double ALLOWABLE_RPM_ERROR = 200;

	private static Flywheel INSTANCE;

	public static Flywheel getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new UpperWheel();
		}

		return INSTANCE;
	}

	private UpperWheel() {
		super();
	}

	@Override
	protected Config getConfig() {
		return new Config() {{
			wheelRadius = WHEEL_RADIUS_METERS;
			gearing = new MotorController.GearingParameters(1.0, WHEEL_RADIUS_METERS, 2048);
			gains = new PIDFController.Gains(0.0, 0.0, 0.0, 0.0);
			feedforward = new MotorFeedforward(0.5509, .0171, 0.0);
			allowableRPMError = ALLOWABLE_RPM_ERROR;
		}};
	}

	@Override
	protected MotorControllerGroup<TalonFX> getMotors() {
		final var motor = MotorControllerFactory.createDefaultTalonFX(SHOOTER_LOWER_WHEEL_PORT);
		motor.setInvertedOutput(false);
		return new MotorControllerGroup<>(motor, List.of());
	}
}
