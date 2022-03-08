package com.gemsrobotics.lib.controls;

import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.RigidTransformWithCurvature;
import com.gemsrobotics.lib.subsystems.drivetrain.WheelState;
import com.gemsrobotics.lib.trajectory.TrajectoryIterator;
import com.gemsrobotics.lib.trajectory.parameterization.TimedState;
import io.github.oblarg.oblog.Loggable;

import java.util.Optional;

public abstract class MotionPlanner {

	public static class MotionConfig implements Loggable {
		public double maxDx; // meters
		public double maxDy; // meters
		public double maxDtheta; // radians
		public double maxVoltage; // volts
		public double maxVelocity; // meters/second
		public double maxAcceleration; // meters/second^2
		public double maxCentripetalAcceleration; // meters/s
	}

	public static class Output implements Loggable {
		public WheelState velocityRadiansPerSecond = new WheelState();
		public WheelState accelerationRadiansPerSecondSquared = new WheelState();
		public WheelState feedforwardVoltage = new WheelState();
	}

	public abstract void setTrajectory(TrajectoryIterator<TimedState<RigidTransformWithCurvature>> trajectory);
	public abstract Optional<Output> update(final double timestamp, final RigidTransform currentPose, final boolean isHighGear);
	public abstract void reset();
}
