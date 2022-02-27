package com.gemsrobotics.frc2019;

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

public class Chassis extends DifferentialDrive<TalonFX> {
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
	protected void onStop(final double timestamp) {
		setDisabled();
	}

	@Override
	protected Config getConfig() {
		return new Config() {{
			maxVoltage = 12.0;
			secondsToMaxVoltage = 0.1;

			gearingLowGear = new MotorController.GearingParameters(1.0, 1.0, 2048);
			gearingHighGear = gearingLowGear;

			gainsLowGear = new PIDFController.Gains(5.33, 0.0, 0.0, 0.0); // 0.69
			gainsHighGear = gainsLowGear;

			propertiesLowGear = new MotorModel.Properties() {{
				speedRadiansPerSecondPerVolt = 1.0;
				torquePerVolt = 1.0;
				stictionVoltage = 1.0;
			}};
			propertiesHighGear = propertiesLowGear;

			propertiesModel = new DifferentialDriveModel.Properties() {{
				massKg = 1.0; // kg
				angularMomentInertiaKgMetersSquared = Math.pow(Units.inches2Meters(6.0), 2) * massKg;
				angularDragTorquePerRadiansPerSecond = 12.0;
				wheelRadiusMeters = 1.0;
//				wheelbaseRadiusMeters = 0.5265; // old
				wheelbaseRadiusMeters = 0.6746875; // new
			}};

			motionConfig = new DriveMotionPlanner.MotionConfig() {{
				beta = 2.0;
				zeta = 0.7;

				maxDx = 0.00127 * 4;
				maxDy = 0.00127;
				maxDtheta = Rotation.degrees(5.0).getRadians();
				maxVoltage = 10.0;
				maxVelocity = 4.8;
				maxAcceleration = 3.2;
				maxCentripetalAcceleration = 2.0;
			}};

			openLoopConfig = new OpenLoopDriveHelper.Config() {{
				quickTurnScalar = 1.0;
			}};
		}};
	}

	@Override
	protected MotorControllerGroup<TalonFX> getMotorControllersLeft() {
		final var master = MotorControllerFactory.createDefaultTalonFX(1);
		final var follower = MotorControllerFactory.createDefaultTalonFX(2);
		final var group = new MotorControllerGroup<TalonFX>(master, List.of(follower));
		master.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		follower.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		group.followMaster(false);
		return group;
	}

	@Override
	protected MotorControllerGroup<TalonFX> getMotorControllersRight() {
		final var master = MotorControllerFactory.createDefaultTalonFX(3);
		final var follower = MotorControllerFactory.createDefaultTalonFX(4);
		final var group = new MotorControllerGroup<TalonFX>(master, List.of(follower));
		master.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		follower.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		master.setInvertedOutput(true);
		group.followMaster(false);
		return group;
	}

	@Override
	protected Transmission getTransmission() {
		return new SingleSpeedTransmission();
	}
}
