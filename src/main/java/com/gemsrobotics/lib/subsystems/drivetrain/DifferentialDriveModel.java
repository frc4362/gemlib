package com.gemsrobotics.lib.subsystems.drivetrain;

import com.gemsrobotics.lib.physics.MotorModel;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Twist;

import java.util.Arrays;

import static com.gemsrobotics.lib.utils.MathUtils.epsilonEquals;
import static com.gemsrobotics.lib.utils.MathUtils.Epsilon;
import static com.gemsrobotics.lib.utils.MathUtils.Bounds;
import static java.lang.Math.*;

public class DifferentialDriveModel {
	public static class Properties {
        // Equivalent mass when accelerating purely linearly, in kg.
        // This is "equivalent" in that it also absorbs the effects of drivetrain inertia.
        // Measure by doing drivetrain acceleration characterization in a straight line.
		public double massKg;

        // Equivalent moment of inertia when accelerating purely angularly, in kg*m^2.
        // This is "equivalent" in that it also absorbs the effects of drivetrain inertia.
        // Measure by doing drivetrain acceleration characterization while turning in place.
		public double momentInertiaKgMetersSquared;

        // Drag torque (proportional to angular velocity) that resists turning, in N*m/rad/s
        // Empirical testing of our drivebase showed that there was an unexplained loss in torque ~proportional to
		// angular velocity, likely due to scrub of wheels.
		public double angularDragTorquePerRadiansPerSecond;

        // Self-explanatory. Measure by rolling the robot a known distance and counting encoder ticks.
		public double wheelRadiusMeters;

        // "Effective" kinematic wheelbase radius.  Might be larger than theoretical to compensate for skid steer.
		// Measure by turning the robot in place several times and figuring out what the equivalent wheelbase radius is.
		public double wheelbaseRadiusMeters;
	}

	public static class Dynamics {
		public double curvatureRadiansPerMeter = 0.0;
		public double dcurvatureRadiansPerMeterSquared = 0.0;

		public ChassisState chassisVelocity = new ChassisState();
		public ChassisState chassisAcceleration = new ChassisState();

		public WheelState wheelVelocityMetersPerSecond = new WheelState();
		public WheelState wheelAccelerationMetersPerSecondSquared = new WheelState();

		public WheelState voltage = new WheelState();
		public WheelState torque = new WheelState();
	}

	public final MotorModel transmissionLow, transmissionHigh;
    public final double
			wheelBaseRadiusMeters,
			angularDragTorquePerRadianPerSecond,
			massKg,
			momentInertiaKgMetersSquared,
			wheelRadiusMeters;

	// two-speed drive train
	public DifferentialDriveModel(final Properties properties, final MotorModel gearLow, final MotorModel gearHigh) {
		wheelBaseRadiusMeters = properties.wheelbaseRadiusMeters;
		angularDragTorquePerRadianPerSecond = properties.angularDragTorquePerRadiansPerSecond;
		massKg = properties.massKg;
		momentInertiaKgMetersSquared = properties.momentInertiaKgMetersSquared;
		wheelRadiusMeters = properties.wheelRadiusMeters;

		transmissionLow = gearLow;
		transmissionHigh = gearHigh;
	}

    // single speed drive train
    public DifferentialDriveModel(final Properties properties, final MotorModel transmission) {
        this(properties, transmission, transmission);
    }

	private MotorModel getTransmission(final boolean isHighGear) {
	    if (isHighGear) {
            return transmissionHigh;
        } else {
            return transmissionLow;
        }
    }

	// Left/right to linear/angular (wheel)
	// Input/demand could be either getVelocity or getAcceleration...the math is the same.
	public ChassisState forwardKinematics(final WheelState wheels) {
	    final var ret = new ChassisState();
	    ret.linearMeters = (wheels.left + wheels.right) / 2 * wheelRadiusMeters;
	    ret.angularRadians = (wheels.right - wheels.left) / (2 * wheelBaseRadiusMeters) * wheelRadiusMeters;
		return ret;
	}

	/**
	 * Forward kinematics using only encoders, rotation is implicit (less accurate than below, but useful for predicting motion)
	 */
	public Twist forwardKinematics(final double leftDeltaDistance, final double rightDeltaDistance) {
		final var thetaDelta = (rightDeltaDistance - leftDeltaDistance) / wheelBaseRadiusMeters;
		return forwardKinematics(leftDeltaDistance, rightDeltaDistance, thetaDelta);
	}

	/**
	 * Forward kinematics using encoders and explicitly measured rotation (ex. from gyro)
	 */
	public Twist forwardKinematics(final double leftDeltaDistance, final double rightDeltaDistance, final double thetaDeltaRadians) {
		final var dx = (leftDeltaDistance + rightDeltaDistance) / 2;
		return new Twist(dx, 0.0, thetaDeltaRadians);
	}

	/**
	 * For convenience, forward kinematics with an absolute rotation and previous rotation.
	 */
	public Twist forwardKinematics(
			final Rotation previousRotation,
			final double leftDeltaDistance,
			final double rightDeltaDistance,
			final Rotation currentRotation
	) {
		return forwardKinematics(leftDeltaDistance, rightDeltaDistance, currentRotation.difference(previousRotation).getRadians());
	}

	public RigidTransform solveForwardKinematics(final RigidTransform currentPose, final Twist delta) {
		return currentPose.transformBy(delta.toRigidTransform());
	}

	public RigidTransform solveForwardKinematics(
			final RigidTransform currentPose,
			final double leftDeltaDistance,
			final double rightDeltaDistance,
			final Rotation currentHeading
	) {
		final var withGyro = forwardKinematics(currentPose.getRotation(), leftDeltaDistance, rightDeltaDistance, currentHeading);
		return solveForwardKinematics(currentPose, withGyro);
	}

	// Linear/angular (m/s) to left/right (rads/s)
	// Input/demand could be either velocity or acceleration...the math is the same.
    // THIS IS WHERE EVERYTHING GOES FROM METERS TO RADIANS PER SECOND
	public WheelState inverseKinematics(final ChassisState chassis) {
		return new WheelState(
				(chassis.linearMeters - chassis.angularRadians * wheelBaseRadiusMeters) / wheelRadiusMeters,
				(chassis.linearMeters + chassis.angularRadians * wheelBaseRadiusMeters) / wheelRadiusMeters
		);
	}

	// Linear/angular velocity and left/right voltage to linear/angular acceleration
	public Dynamics solveForwardDynamics(final ChassisState velocityMetersPerSecond, final WheelState voltage, final boolean isHighGear) {
		final var transmission = getTransmission(isHighGear);

		final var ret = new Dynamics();
		ret.wheelVelocityMetersPerSecond = inverseKinematics(velocityMetersPerSecond);
		ret.chassisVelocity = velocityMetersPerSecond;
		ret.voltage = voltage;

		// not moving, and not giving enough torque to start moving
		final boolean stuckLeft = epsilonEquals(ret.wheelVelocityMetersPerSecond.left, 0) && abs(voltage.left) < transmission.stictionVoltage;
		final boolean stuckRight = epsilonEquals(ret.wheelVelocityMetersPerSecond.right, 0) && abs(voltage.right) < transmission.stictionVoltage;

		if (stuckLeft && stuckRight) {
			return ret;
		}

		ret.curvatureRadiansPerMeter = ret.chassisVelocity.angularRadians / ret.chassisVelocity.linearMeters;

		if (Double.isNaN(ret.curvatureRadiansPerMeter)) {
			ret.curvatureRadiansPerMeter = 0.0;
		}

		ret.torque = new WheelState(
				transmission.torqueForVoltage(ret.wheelVelocityMetersPerSecond.left, voltage.left),
				transmission.torqueForVoltage(ret.wheelVelocityMetersPerSecond.right, voltage.right)
		);

		ret.chassisAcceleration.linearMeters = (ret.torque.left + ret.torque.right) / (wheelRadiusMeters * massKg);
		ret.chassisAcceleration.angularRadians = wheelBaseRadiusMeters * (ret.torque.right - ret.torque.left) / (wheelRadiusMeters * momentInertiaKgMetersSquared)
                - (velocityMetersPerSecond.angularRadians * angularDragTorquePerRadianPerSecond / momentInertiaKgMetersSquared);

		ret.dcurvatureRadiansPerMeterSquared =
                (ret.chassisAcceleration.angularRadians - ret.chassisAcceleration.linearMeters * ret.curvatureRadiansPerMeter)
                / (ret.chassisVelocity.linearMeters * ret.chassisVelocity.linearMeters);

		if (Double.isNaN(ret.dcurvatureRadiansPerMeterSquared)) {
			ret.dcurvatureRadiansPerMeterSquared = 0.0;
		}

		ret.wheelAccelerationMetersPerSecondSquared.left = ret.chassisAcceleration.linearMeters - ret.chassisAcceleration.angularRadians * wheelBaseRadiusMeters;
		ret.wheelAccelerationMetersPerSecondSquared.right = ret.chassisAcceleration.linearMeters + ret.chassisAcceleration.angularRadians * wheelBaseRadiusMeters;

		return ret;
	}

	public Dynamics solveForwardDynamics(final WheelState velocityMetersPerSecond, final WheelState voltage, final boolean isHighGear) {
		return solveForwardDynamics(forwardKinematics(velocityMetersPerSecond), voltage, isHighGear);
	}

	// Linear/angular velocity and linear/angular acceleration to left/right voltage
	public Dynamics solveInverseDynamics(final ChassisState velocity, final ChassisState acceleration, final boolean isHighGear) {
		final var ret = new Dynamics();

		ret.chassisVelocity = velocity;
		ret.curvatureRadiansPerMeter = ret.chassisVelocity.angularRadians / ret.chassisVelocity.linearMeters;

		if (Double.isNaN(ret.curvatureRadiansPerMeter)) {
			ret.curvatureRadiansPerMeter = 0.0;
		}

		ret.chassisAcceleration = acceleration;
		ret.dcurvatureRadiansPerMeterSquared = (ret.chassisAcceleration.angularRadians - ret.chassisAcceleration.linearMeters * ret.curvatureRadiansPerMeter)
                / (ret.chassisVelocity.linearMeters * ret.chassisVelocity.linearMeters);

		if (Double.isNaN(ret.dcurvatureRadiansPerMeterSquared)) {
			ret.dcurvatureRadiansPerMeterSquared = 0.0;
		}

		ret.wheelVelocityMetersPerSecond = inverseKinematics(ret.chassisVelocity);
		ret.wheelAccelerationMetersPerSecondSquared = inverseKinematics(ret.chassisAcceleration);

		ret.torque = new WheelState(
				0.5 * wheelRadiusMeters * ((acceleration.linearMeters * massKg)
                        - (acceleration.angularRadians * momentInertiaKgMetersSquared / wheelBaseRadiusMeters)
                        - (velocity.angularRadians * angularDragTorquePerRadianPerSecond / wheelBaseRadiusMeters)),
				0.5 * wheelRadiusMeters * ((acceleration.linearMeters * massKg)
                        + (acceleration.angularRadians * momentInertiaKgMetersSquared / wheelBaseRadiusMeters)
                        + (velocity.angularRadians * angularDragTorquePerRadianPerSecond / wheelBaseRadiusMeters))
		);

        final var transmission = getTransmission(isHighGear);
		ret.voltage.left = transmission.voltageFromTorque(ret.wheelVelocityMetersPerSecond.left, ret.torque.left);
		ret.voltage.right = transmission.voltageFromTorque(ret.wheelVelocityMetersPerSecond.right, ret.torque.right);
		return ret;
	}

	public Dynamics solveInverseDynamics(final WheelState velocity, final WheelState acceleration, final boolean isHighGear) {
		return solveInverseDynamics(forwardKinematics(velocity), forwardKinematics(acceleration), isHighGear);
	}

	public double calculateMaxVelocity(final double curvatureRadiansPerMeter, final double maxVoltage, final boolean isHighGear) {
		final var transmission = getTransmission(isHighGear);
		final double freeSpeedRadiansPerSecond = transmission.freeSpeedAtVoltageRadiansPerSecond(maxVoltage);

		if (epsilonEquals(curvatureRadiansPerMeter, 0)) {
			return wheelRadiusMeters * freeSpeedRadiansPerSecond;
		} else if (Double.isInfinite(curvatureRadiansPerMeter)) {
			return signum(curvatureRadiansPerMeter) * wheelRadiusMeters * freeSpeedRadiansPerSecond / wheelBaseRadiusMeters;
		}

		final double rightConstrainedRadiansPerSecond = freeSpeedRadiansPerSecond
                * (1.0 + wheelBaseRadiusMeters * curvatureRadiansPerMeter)
                / (1.0 - wheelBaseRadiusMeters * curvatureRadiansPerMeter);

		if (abs(rightConstrainedRadiansPerSecond) < freeSpeedRadiansPerSecond) {
			return wheelRadiusMeters * (freeSpeedRadiansPerSecond + rightConstrainedRadiansPerSecond) / 2.0;
		}

		final double leftConstrainedRadiansPerSecond = freeSpeedRadiansPerSecond
                * (1.0 - wheelBaseRadiusMeters * curvatureRadiansPerMeter)
                / (1.0 + wheelBaseRadiusMeters * curvatureRadiansPerMeter);

		return wheelRadiusMeters * (freeSpeedRadiansPerSecond + leftConstrainedRadiansPerSecond) / 2.0;
	}

	// Calculate the minimum and maximum forwards (linear) acceleration along the same curvature.
	public Bounds calculateMinMaxAcceleration(
			final ChassisState velocity,
			final double curvatureRadiansPerMeter,
			final double maxVoltage,
            final boolean isHighGear
	) {
		final var transmission = getTransmission(isHighGear);
		final var result = new Bounds(Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY);

		final WheelState wheelVelocitiesMetersPerSecond = inverseKinematics(velocity);

		// Math:
		// (Tl + Tr) / r_w = m*a
		// (Tr - Tl) / r_w * r_wb - drag*w = i*(a * k + v^2 * dk)

		// 2 equations, 2 unknowns.
		// Solve for a and (Tl|Tr)

		final double linearTorque = Double.isInfinite(curvatureRadiansPerMeter) ? 0.0 : massKg * wheelBaseRadiusMeters;
		final double angularTorque = Double.isInfinite(curvatureRadiansPerMeter) ? momentInertiaKgMetersSquared : momentInertiaKgMetersSquared * curvatureRadiansPerMeter;
		final double dragTorque = velocity.angularRadians * angularDragTorquePerRadianPerSecond;

		// Check all four cases and record the min and max valid accelerations.
		for (final boolean left : Arrays.asList(false, true)) {
			for (final double sign : Arrays.asList(1.0, -1.0)) {
				final var wheelSpeedMetersPerSecond = left ? wheelVelocitiesMetersPerSecond.left : wheelVelocitiesMetersPerSecond.right;
				final var fixedTorque = transmission.torqueForVoltage(wheelSpeedMetersPerSecond, sign * maxVoltage);

				final double variableTorque;

				// NOTE: variableTorque is wrong.  Units don't work out correctly.  We made a math error somewhere...
				// Leaving this "as is" for code release so as not to be disingenuous, but this whole function needs
				// revisiting in the future... its probably good enough lol
				if (left) {
					variableTorque = ((-dragTorque) * massKg * wheelRadiusMeters + fixedTorque * (linearTorque + angularTorque))
                            / (linearTorque - angularTorque);
				} else {
					variableTorque = ((+dragTorque) * massKg * wheelRadiusMeters + fixedTorque * (linearTorque - angularTorque))
                            / (linearTorque + angularTorque);
				}

				final double otherWheelSpeedMetersPerSecond = left ? wheelVelocitiesMetersPerSecond.right : wheelVelocitiesMetersPerSecond.left;
				final double variableVoltage = transmission.torqueForVoltage(otherWheelSpeedMetersPerSecond, variableTorque);

				if (abs(variableVoltage) <= maxVoltage + Epsilon) {
				    final double accelerationMetersPerSecondSquared;

					if (Double.isInfinite(curvatureRadiansPerMeter)) {
						final double drag = (dragTorque / momentInertiaKgMetersSquared);
						final double direction = (left ? -1.0 : 1.0);
						final double torqueDifferential = fixedTorque - variableTorque;
						accelerationMetersPerSecondSquared = (direction * torqueDifferential * wheelBaseRadiusMeters / (momentInertiaKgMetersSquared * wheelRadiusMeters)) - drag;
					} else {
						accelerationMetersPerSecondSquared = (fixedTorque + variableTorque) / (massKg * wheelRadiusMeters);
					}

					result.min = min(result.min, accelerationMetersPerSecondSquared);
					result.max = max(result.max, accelerationMetersPerSecondSquared);
				}
			}
		}

		return result;
	}
}