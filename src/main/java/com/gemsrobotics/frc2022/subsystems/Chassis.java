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
	private final static double STALL_TORQUE = 4.69; // nM
	private final static double TORQUE_EFFICIENCY = 0.95; // magic
	private final static int MOTOR_COUNT = 4;

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
		final double wheelRadius = Units.inches2Meters(4.0) / 2.0;
		final double freeSpeed = 4.6 / wheelRadius; // radians/s
		final double trackWidth = 0.534; // m
		final double mass = 62.414; // kg
		final double gearing = 5.91; // 5.91 : 1
		final double kS = 0.9; // V
		final double kV = 0.10622;//12.0 / freeSpeed; // V / (rad / s)
		final double analyticalLinearKA = 12.0 / ((gearing * STALL_TORQUE * TORQUE_EFFICIENCY * MOTOR_COUNT) / (mass * wheelRadius * wheelRadius));
		// V / (rad / s^2)
		final double angularKa = 2.7;
		final double linearKa = 0.024337;

		return new Config() {{
			maxVoltage = peakVoltage;
			secondsToMaxVoltage = 0.1;

			gearingLowGear = new MotorController.GearingParameters(1.0 / gearing, wheelRadius, 2048);
			gearingHighGear = gearingLowGear;

			gainsLowGear = new PIDFController.Gains(0.04, 0.0, 0.0, 0.0);
			gainsHighGear = gainsLowGear;

			propertiesLowGear = new MotorModel.Properties() {{
				speedRadiansPerSecondPerVolt = (1 / kV);
				torquePerVolt = wheelRadius * wheelRadius * mass / (2.0 * linearKa);
				stictionVoltage = kS;
			}};
			propertiesHighGear = propertiesLowGear;

			propertiesModel = new DifferentialDriveModel.Properties() {{
				massKg = mass; // kg
				angularMomentInertiaKgMetersSquared = (mass * trackWidth * angularKa) / (linearKa * 2.0);
				angularDragTorquePerRadiansPerSecond = 12.0;
				wheelRadiusMeters = wheelRadius;
				wheelbaseRadiusMeters = trackWidth / 2.0;
			}};

			motionConfig = new DriveMotionPlanner.MotionConfig() {{
				beta = 2.0;
				zeta = 0.7;

				maxDx = 0.00127 * 4; // m
				maxDy = 0.00127; // m
				maxDtheta = Rotation.degrees(5.0).getRadians();
				maxVoltage = 10.0; // V
				maxVelocity = 4.6; // m / s
				maxAcceleration = 4.0; // m / s^2
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
