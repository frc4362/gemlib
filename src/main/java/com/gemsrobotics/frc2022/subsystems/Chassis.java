package com.gemsrobotics.frc2022.subsystems;

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
	private static final StatorCurrentLimitConfiguration CURRENT_LIMIT = new StatorCurrentLimitConfiguration(
			true,
			60,
			60,
			0.2);

	private static Chassis INSTANCE;

	public static Chassis getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Chassis();
		}

		return INSTANCE;
	}

	private Chassis() {
		super();
	}

	@Override
	protected Config getConfig() {
		final double peakVoltage = 12.0;
		final double wheelRadius = Units.inches2Meters(3.8) / 2.0;
		final double freeSpeed = 91.64774179196561; // radians/s
		final double kS = 0.36167; // V
		final double kV = 0.1329; // V / (rad / s)
		final double kA = 0.012; // V / (rad / s^2)
		final double mass = 62.73; // kg

		return new Config() {{
			maxVoltage = peakVoltage;
			secondsToMaxVoltage = 0.1;

			gearingLowGear = new MotorController.GearingParameters(1.0 / 5.91, wheelRadius, 2048);
			gearingHighGear = gearingLowGear;

			gainsLowGear = new PIDFController.Gains(0.0, 0.0, 0.0, 0.0);
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
				wheelbaseRadiusMeters = 0.6746875; // new
			}};

			motionConfig = new DriveMotionPlanner.MotionConfig() {{
				beta = 2.0;
				zeta = 0.7;

				maxDx = 0.00127 * 4; // m
				maxDy = 0.00127; // m
				maxDtheta = Rotation.degrees(5.0).getRadians();
				maxVoltage = 10.0; // V
				maxVelocity = 4.8; // m / s
				maxAcceleration = 3.2; // m / s^2
				maxCentripetalAcceleration = 2.0; // m / s^2
			}};

			openLoopConfig = new OpenLoopDriveHelper.Config() {{
				quickTurnScalar = 0.5;
			}};
		}};
	}

	private void configMotors(final MotorControllerGroup<TalonFX> motors) {
		motors.forEach(motor -> {
			motor.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
			motor.getInternalController().configVoltageCompSaturation(12.0);
			motor.getInternalController().enableVoltageCompensation(true);
			motor.getInternalController().configStatorCurrentLimit(CURRENT_LIMIT);
		});
	}

	@Override
	protected MotorControllerGroup<TalonFX> getMotorControllersLeft() {
		final var master = MotorControllerFactory.createDefaultTalonFX(0);
		master.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		final var slave = MotorControllerFactory.createSlaveTalonFX(1);
		slave.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		final var group = new MotorControllerGroup<>(master, List.of(slave));
		configMotors(group);
		group.getMaster().setInvertedOutput(true);
		return group;
	}

	@Override
	protected MotorControllerGroup<TalonFX> getMotorControllersRight() {
		final var master = MotorControllerFactory.createDefaultTalonFX(2);
		master.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		final var slave = MotorControllerFactory.createSlaveTalonFX(3);
		slave.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		final var group = new MotorControllerGroup<>(master, List.of(slave));
		configMotors(group);
		group.getMaster().setInvertedOutput(false);
		group.getMaster().getInternalController().setInverted(false);
		return group;
	}

	@Override
	protected Transmission getTransmission() {
		return new SingleSpeedTransmission();
	}

	public Rotation getPitch() {
		return m_imu.getPitch();
	}

	@Override
	protected void onStop(double timestamp) {

	}
}
