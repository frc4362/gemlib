package com.gemsrobotics.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.controls.DriveMotionPlanner;
import com.gemsrobotics.lib.controls.PIDFController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerGroup;
import com.gemsrobotics.lib.drivers.transmission.SingleSpeedTransmission;
import com.gemsrobotics.lib.drivers.transmission.Transmission;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.physics.MotorModel;
import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDrive;
import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDriveModel;
import com.gemsrobotics.lib.subsystems.drivetrain.OpenLoopDriveHelper;
import com.gemsrobotics.lib.utils.Units;

import java.util.List;
import java.util.Objects;

public final class Chassis extends DifferentialDrive<TalonFX> {
	private static Chassis INSTANCE;

	public static Chassis getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Chassis();
		}

		return INSTANCE;
	}

	@Override
	protected Config getConfig() {
		final double peakVoltage = 12.0;
		final double wheelRadius = 0.08016875;
		final double freeSpeed = 4.8 / wheelRadius; // m/s
		final double kS = 0.25; // V
		final double kV = 0.06875 * 2.0; // V / (rad / s)
		final double kA = 0.011; // V / (rad / s^2)
		final double mass = 66.678; // kg

		return new Config() {{
			maxVoltage = peakVoltage;
			secondsToMaxVoltage = 0.1;

			gearingLowGear = new MotorController.GearingParameters(1.0 / 8.751, wheelRadius, 2048);
			gearingHighGear = gearingLowGear;

			gainsLowGear = new PIDFController.Gains(5.33, 0.0, 0.0, 0.0); // 0.69
			gainsHighGear = gainsLowGear;

			propertiesLowGear = new MotorModel.Properties() {{
				speedRadiansPerSecondPerVolt = (1 / kV);
				torquePerVolt = wheelRadius * wheelRadius * mass / (2.0 * kA);
				stictionVoltage = kS;
			}};
			propertiesHighGear = propertiesLowGear;

			propertiesModel = new DifferentialDriveModel.Properties() {{
				massKg = mass; // kg
				angularMomentInertiaKgMetersSquared = Math.pow(Units.inches2Meters(6.0), 2) * massKg;
				angularDragTorquePerRadiansPerSecond = 12.0;
				wheelRadiusMeters = wheelRadius;
				wheelbaseRadiusMeters = 0.351;
			}};

			motionConfig = new DriveMotionPlanner.MotionConfig() {{
				beta = 2.0;
				zeta = 0.7;

				maxDx = 0.00127;
				maxDy = 0.00127;
				maxDtheta = Rotation.degrees(5.0).getRadians();
				maxVoltage = 10.0;
				maxVelocity = 4.8;
				maxAcceleration = 3.2;
				maxCentripetalAcceleration = 2.0;
			}};

			openLoopConfig = new OpenLoopDriveHelper.Config() {{
				useSineAttack = true;
				zNonLinearityHighGear = 1.0;
				zNonLinearityLowGear = 0.85;

				sensitivityHighGear = 0.65;
				sensitivityLowGear = 0.7;

				negativeInertiaScalarHigh = 4.0;
				negativeInertiaThresholdLow = 0.65;
				negativeInertiaTurnScalarLow = 3.5;
				negativeInertiaCloseScalarLow = 4.0;
				negativeInertiaFarScalarLow = 5.0;

				quickStopDeadband = 0.3; // 0.5
				quickStopScalar = 3.0; // 5.0
				quickStopWeight = 0.1;
			}};
		}};
	}

	private void configMotors(final MotorControllerGroup<TalonFX> motors) {
		motors.forEach(motor -> {
			motor.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
			motor.getInternalController().enableVoltageCompensation(true);
			motor.getInternalController().configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
					true,
					75,
					0,
					0.02),
				10);
		});
	}

	@Override
	protected MotorControllerGroup<TalonFX> getMotorControllersLeft() {
		final var master = MotorControllerFactory.createDefaultTalonFX(1);
		final var slave = MotorControllerFactory.createSlaveTalonFX(2);
		final var group = new MotorControllerGroup<>(master, List.of(slave));
		configMotors(group);
		group.getMaster().setInvertedOutput(false);
		return group;
	}

	@Override
	protected MotorControllerGroup<TalonFX> getMotorControllersRight() {
		final var master = MotorControllerFactory.createDefaultTalonFX(3);
		final var slave = MotorControllerFactory.createSlaveTalonFX(4);
		final var group = new MotorControllerGroup<>(master, List.of(slave));
		configMotors(group);
		group.getMaster().setInvertedOutput(true);
		return group;
	}

	@Override
	protected Transmission getTransmission() {
		return new SingleSpeedTransmission();
	}

	@Override
	protected void onStop(double timestamp) {
		report("Stopped");
	}
}
