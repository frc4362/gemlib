package com.gemsrobotics.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.controls.MotorFeedforward;
import com.gemsrobotics.lib.controls.PIDFController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerGroup;
import com.gemsrobotics.lib.subsystems.Flywheel;
import com.gemsrobotics.lib.utils.MathUtils;
import com.gemsrobotics.lib.utils.Units;

import java.util.List;
import java.util.Objects;

public final class LowerWheel extends Flywheel {
	private static final int SHOOTER_LOWER_WHEEL_PORT = 8;
	private static final double WHEEL_RADIUS_METERS = Units.inches2Meters(3.75 / 2.0);
	private static final double ALLOWABLE_RPM_ERROR = 150;

	private static Flywheel INSTANCE;

	public static Flywheel getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new LowerWheel();
		}

		return INSTANCE;
	}

	private LowerWheel() {
		super();
	}

	@Override
	protected Config getConfig() {
		return new Config() {{
			wheelRadius = WHEEL_RADIUS_METERS;
			gearing = new MotorController.GearingParameters(1.0, WHEEL_RADIUS_METERS, 2048);
			gains = new PIDFController.Gains(0.1, 0.0, 0.0, 0.0);
			feedforward = new MotorFeedforward(0.0, 0.017732, 0.00132);
			allowableRPMError = ALLOWABLE_RPM_ERROR;
			currentLimitConfiguration = new StatorCurrentLimitConfiguration(false, 100, 100, 0.1);
		}};
	}

	@Override
	protected MotorControllerGroup<TalonFX> getMotors() {
		final var motor = MotorControllerFactory.createHighPerformanceTalonFX(SHOOTER_LOWER_WHEEL_PORT);
		motor.setInvertedOutput(true);
		return new MotorControllerGroup<>(motor, List.of());
	}
}
