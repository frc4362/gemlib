package com.gemsrobotics.frc2019.subsystems.drive;

import com.gemsrobotics.frc2019.OperatorInterface;
import com.gemsrobotics.frc2019.commands.DriveCommand;
import com.gemsrobotics.frc2019.commands.ChassisLocalization;
import com.gemsrobotics.frc2019.commands.ShiftScheduler;
import com.gemsrobotics.frc2019.util.DualTransmission;
import com.gemsrobotics.frc2019.util.MyAHRS;
import com.gemsrobotics.frc2019.util.PIDF;
import com.gemsrobotics.frc2019.util.camera.Limelight;
import com.gemsrobotics.frc2019.util.motion.Pose;
import com.gemsrobotics.frc2019.util.motion.Rotation;
import com.gemsrobotics.frc2019.util.motion.Twist;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import static com.gemsrobotics.frc2019.util.motion.EpsilonValue.Epsilon;
import static java.lang.Math.*;
import static java.lang.Math.min;

@SuppressWarnings({"unused", "WeakerAccess"})
public final class DifferentialDrive extends Subsystem implements Sendable {
	public static final double dt = 0.02;

	private static final double METER_TO_INCHES = 0.0254;
	private static final double DEGREES_THRESHOLD = 3.5;

	private final Kinematics m_kinematics;
	private final Specifications m_specifications;
	private final DualTransmission m_transmission;
	private final RobotState m_state;
	private final List<CANSparkMax> m_motors;
	private final MyAHRS m_ahrs;
	private final ShiftScheduler m_shiftScheduler;

	private DriveCommand m_driveCommand;
	private double m_accumulator;
	private String m_name;

	private static final String[] MOTOR_CONTROLLER_NAMES = {
		"Left Front", "Left Back", "Right Front", "Right Back"
	};

	private ChassisLocalization m_robotStateEstimator;

	public DifferentialDrive(
			final DrivePorts drivePorts,
			final Specifications specifications,
			final Solenoid shifter,
			final MyAHRS ahrs,
			final boolean useVelocityControl
	) {
		final Integer[] ports = drivePorts.get();
		m_ahrs = ahrs;
		m_specifications = specifications;
		m_transmission = new DualTransmission(shifter);
		m_kinematics = new Kinematics(m_specifications, this);
		m_state = new RobotState(this);
		m_robotStateEstimator = new ChassisLocalization(this);
		m_shiftScheduler = new ShiftScheduler(this);

		if (ports.length != 4) {
			throw new RuntimeException("Wrong number of ports!");
		}

		m_motors = Arrays.stream(ports)
				.map(port -> new CANSparkMax(port, MotorType.kBrushless))
				.collect(Collectors.toList());

		m_motors.get(1).follow(m_motors.get(0));
		m_motors.get(3).follow(m_motors.get(2));
		m_motors.forEach(m -> {
			m.setIdleMode(CANSparkMax.IdleMode.kBrake);
			m.setInverted(false);
		});

		m_motors.stream().map(CANSparkMax::getPIDController).forEach(controller -> {
			m_specifications.pidDrive.configure(controller);
			controller.setOutputRange(-1.0, +1.0);
		});

		m_accumulator = 0.0;
	}

	public void configureDriveCommand(
			final Limelight limelight,
			final OperatorInterface controls,
			final SendableChooser<Boolean> toggler
	) {
		m_driveCommand = new DriveCommand(this, limelight, controls, toggler);
	}

	public void drive(final double leftPower, final double rightPower) {
		getMotor(Side.LEFT).set(leftPower);
		getMotor(Side.RIGHT).set(-rightPower);
	}

	private static final double LIMIT = 1.0;

	private static double limit(final double v) {
		return abs(v) < LIMIT ? v : LIMIT * signum(v);
	}

	private static double constrain(
			final double bot,
			final double val,
			final double top
	) {
		return max(bot, min(val, top));
	}

	public void drive(final DrivePower velocity) {
		curvatureDrive(velocity.linear(), velocity.angular(), false);
	}

	public void curvatureDrive(
			final double linearPower,
			double zRotation,
			final boolean isQuickTurn
	) {
		double overPower, angularPower;

		if (isQuickTurn) {
			if (Math.abs(linearPower) < m_specifications.quickstopThreshold) {
				m_accumulator = (1 - m_specifications.alpha) * m_accumulator + m_specifications.alpha * limit(zRotation) * 2;
			}

			overPower = 1.0;
			angularPower = -zRotation;
		} else {
			overPower = 0.0;
			zRotation *= -signum(linearPower);
			angularPower = abs(linearPower) * zRotation * m_specifications.turnSensitivity - m_accumulator;

			if (m_accumulator > 1) {
				m_accumulator -= 1;
			} else if (m_accumulator < -1) {
				m_accumulator += 1;
			} else {
				m_accumulator = 0.0;
			}
		}

		double leftPower = linearPower - angularPower,
				rightPower = linearPower + angularPower;

		if (leftPower > 1.0) {
			rightPower -= overPower * (leftPower - 1.0);
			leftPower = 1.0;
		} else if (rightPower > 1.0) {
			leftPower -= overPower * (rightPower - 1.0);
			rightPower = 1.0;
		} else if (leftPower < -1.0) {
			rightPower += overPower * (-1.0 - leftPower);
			leftPower = -1.0;
		} else if (rightPower < -1.0) {
			leftPower += overPower * (-1.0 - rightPower);
			rightPower = -1.0;
		}

		drive(leftPower, rightPower);
	}

	public boolean turnToHeading(final double goal) {
		final var currentAngle = m_ahrs.getHalfAngle();

		double error = -(currentAngle - goal);

		if (abs(error) > 180) {
			error = 360 - error;

			if (currentAngle < 0 && goal > 0) {
				error *= -1;
			}
		}

		final boolean isAtHeading = abs(error) < DEGREES_THRESHOLD;

		if (!isAtHeading) {
			final double angularPower = error * m_specifications.kP_Rotational + copySign(m_specifications.kFF_Rotational, error);
			curvatureDrive(0, constrain(-1, angularPower, 1), true);
		}

		return isAtHeading;
	}

	public void stopMotors() {
		m_motors.forEach(CANSparkMax::stopMotor);
	}

	public List<CANSparkMax> getMotors() {
		return m_motors;
	}

	public CANSparkMax getMotor(final Side side) {
		return m_motors.get(side.idx);
	}

	private double rpm2InPerS(final double rpm) {
		final double rps = rpm / 60.0;
		return rps * m_specifications.rotationsToInches(m_transmission);
	}

	public double getInchesPerSecond(final Side side) {
		return rpm2InPerS(getMotor(side).getEncoder().getVelocity())
			   * side.encoderMultiplier;
	}

	public double getInchesPosition(final Side side) {
		return m_specifications.rotationsToInches(m_transmission)
			   * getMotor(side).getEncoder().getPosition()
			   * side.encoderMultiplier;
	}

	public Kinematics getKinematics() {
		return m_kinematics;
	}

	public Specifications getLocals() {
		return m_specifications;
	}

	public RobotState getState() {
		return m_state;
	}

	public DualTransmission getTransmission() {
		return m_transmission;
	}

	public MyAHRS getAHRS() {
		return m_ahrs;
	}

	public DriveCommand getDriveCommand() {
		return m_driveCommand;
	}

	public ChassisLocalization getStateEstimator() {
		return m_robotStateEstimator;
	}

	public ShiftScheduler getShiftScheduler() {
		return m_shiftScheduler;
	}

	public enum Side {
		LEFT(0, 1), RIGHT(2, -1);

		protected final int idx, encoderMultiplier;

		Side(final int i, final int mult) {
			idx = i;
			encoderMultiplier = mult;
		}

		private static Side forIndex(final int i) {
			if (i < 2) {
				return LEFT;
			} else {
				return RIGHT;
			}
		}
	}

	public static class Specifications {
		public double width, length, wheelDiameter, maxVelocity,
				maxAcceleration, maxJerk, quickstopThreshold, turnSensitivity,
				alpha, kP_Rotational, kFF_Rotational;

		public PIDF pidDrive;

		public double wheelCircumference() {
			return Math.PI * wheelDiameter;
		}

		public double wheelRadius() {
			return wheelDiameter / 2.0;
		}

		public double rotationsToInches(final DualTransmission transmission) {
			return wheelCircumference() / transmission.get().ratio;
		}
	}

	public static class DrivePower {
		private final double linear, angular;

		private DrivePower(final double l, final double a) {
			linear = l;
			angular = a;
		}

		public double linear() {
			return linear;
		}

		public double angular() {
			return angular;
		}

		public double left() {
			return linear - angular;
		}

		public double right() {
			return linear + angular;
		}
	}

	public DrivePower wheelVelocity(final double l, final double r) {
		final double linear = (m_specifications.wheelRadius() * (l + r)) / 2.0,
				angular = m_specifications.wheelRadius() * (r - l) / m_specifications.width;

		return new DrivePower(linear, angular);
	}

	public static DrivePower driveVelocity(final double linear, final double angular) {
		return new DrivePower(linear, angular);
	}

	public static class Kinematics {
		private final Specifications m_specifications;
		private final DifferentialDrive m_chassis;

		public Kinematics(final Specifications specifications, final DifferentialDrive chassis) {
			m_specifications = specifications;
			m_chassis = chassis;
		}

		public Twist forwardKinematics(
				final double wheelDeltaLeft,
				final double wheelDeltaRight
		) {
			final double velocityDelta = (wheelDeltaRight - wheelDeltaLeft) / 2.0,
					rotationDelta = velocityDelta * 2 / m_specifications.width;

			return forwardKinematics(wheelDeltaLeft, wheelDeltaRight, rotationDelta);
		}

		public Twist forwardKinematics(
				final double wheelDeltaLeft,
				final double wheelDeltaRight,
				final double rotationDelta
		) {
			final double dx = (wheelDeltaLeft + wheelDeltaRight) / 2.0;
			return new Twist(dx, 0, rotationDelta);
		}

		public Twist forwardKinematics(
				final Rotation previousHeading,
				final double wheelLeftDelta,
				final double wheelRightDelta,
				final Rotation currentHeading
		) {
			final double dx = (wheelLeftDelta + wheelRightDelta) / 2.0,
					dy = 0.0;

			return new Twist(
					dx,
					dy,
					previousHeading.inverse().rotate(currentHeading).getRadians());
		}

		public Pose integrateForwardKinematics(
				final Pose currentPose,
				final Twist forwardKinematics
		) {
			return currentPose.transform(Pose.exp(forwardKinematics));
		}

		public DrivePower inverseKinematics(final Twist velocity) {
			if (Math.abs(velocity.dtheta) < Epsilon) {
				return m_chassis.wheelVelocity(velocity.dx, velocity.dy);
			} else {
				final double dv = m_specifications.width * velocity.dtheta / 2.0;
				return m_chassis.wheelVelocity(velocity.dx - dv, velocity.dx + dv);
			}
		}
	}

	@Override
	public void initDefaultCommand() { }

	@Override
	public void initSendable(final SendableBuilder builder) {
		builder.setSmartDashboardType("West Coast Drive");

		final List<Runnable> updaters = IntStream.range(0, 4).<Runnable>mapToObj(i -> {
			final CANSparkMax spark = m_motors.get(i);
			final String name = MOTOR_CONTROLLER_NAMES[i];

			final NetworkTableEntry
					setSpeedEntry = builder.getEntry(name + " Setpoint"),
					velocityEntry = builder.getEntry(name + " Velocity (In-s)"),
					positionEntry = builder.getEntry(name + " Position (In)");

			final String id = Integer.toString(spark.getDeviceId());
			builder.getEntry(name + " CAN ID").setString(id);

			return () -> {
				setSpeedEntry.setDouble(spark.get());
				velocityEntry.setDouble(getInchesPerSecond(Side.forIndex(i)));
				positionEntry.setDouble(getInchesPosition(Side.forIndex(i)));
			};
		}).collect(Collectors.toList());

		builder.setUpdateTable(() ->
			  updaters.forEach(Runnable::run));
	}
}
