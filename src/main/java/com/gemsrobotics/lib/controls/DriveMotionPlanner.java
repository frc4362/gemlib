package com.gemsrobotics.lib.controls;

import com.gemsrobotics.lib.subsystems.drivetrain.ChassisState;
import com.gemsrobotics.lib.subsystems.drivetrain.Model;
import com.gemsrobotics.lib.subsystems.drivetrain.WheelState;
import com.gemsrobotics.lib.telemetry.reporting.Reportable;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.RigidTransformWithCurvature;
import com.gemsrobotics.lib.trajectory.TrajectoryIterator;
import com.gemsrobotics.lib.trajectory.parameterization.*;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import java.util.Objects;
import java.util.Optional;

import static com.gemsrobotics.lib.utils.MathUtils.Epsilon;
import static com.gemsrobotics.lib.utils.MathUtils.epsilonEquals;
import static java.lang.Double.isInfinite;
import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

// consider refactoring into a strategy-pattern with a PathController interface
public class DriveMotionPlanner implements Reportable, Loggable {
    @Override
    public String configureLogName() {
        return "Motion Planner";
    }

	public static class MotionConfig implements Loggable {
        @Log(name="Beta (Aggression)(0, +Infinity)")
        public double beta; // aggression coefficient, >0
        @Config(name="Zeta (Dampening)[0, 1]")
        public double zeta; // dampening coefficient, [0, 1]

        @Log(name="Max Dx (m)")
		public double maxDx; // meters
		@Log(name="Max Dy (m)")
        public double maxDy; // meters
		@Log(name="Max Dtheta (rad)")
        public double maxDtheta; // radians
		@Log(name="Max Voltage (V)")
        public double maxVoltage; // volts
		@Log(name="Max Velocity (m per s)")
        public double maxVelocity; // meters/second
		@Log(name="Max Acceleration (m per s^2)")
        public double maxAcceleration; // meters/second^2
		@Log(name="Max Centripetal Acceleration (m per s)")
        public double maxCentripetalAcceleration; // meters/s
	}

	public static class Output implements Loggable {
	    @Log.ToString(name="Velocity (rad per s, rad per s)")
		public WheelState velocity; // rad/s
        @Log.ToString(name="Acceleration (rad per s^2, rad per s^2)")
		public WheelState acceleration; // rad/s^2
        @Log.ToString(name="Feedforward (V, V)")
		public WheelState feedforwardVoltage; // volts

		public void flip() {
			velocity.flip();
			acceleration.flip();
			feedforwardVoltage.flip();
		}
	}

	protected transient final Model m_model;
	protected transient final MotionConfig m_config;

	protected FollowerType m_followerType;

	public enum FollowerType {
	    FEEDFORWARD,
		RAMSETE
	}

	public void setFollowerType(final FollowerType newType) {
		m_followerType = newType;
	}

    @Log.ToString(name="Follower Type (Algorithm)")
	public FollowerType getFollowerType() {
		return m_followerType;
	}

	protected TrajectoryIterator<TimedState<RigidTransformWithCurvature>> m_trajectory;
	@Log.ToString(name="Error (pose)")
	protected RigidTransform m_error;
	@Log.ToString(name="Setpoint (timed pose w/ curvature)")
	protected TimedState<RigidTransformWithCurvature> m_setpoint;
    @Log(name="Reversed? (Boolean)")
	protected boolean m_isReversed;
	protected double m_lastTime;
	protected Output m_output;
	protected ChassisState m_previousVelocity;

	public DriveMotionPlanner(
			final MotionConfig config,
			final Model model,
			final FollowerType followerType
	) {
		m_config = config;
		m_model = model;
		m_followerType = followerType;
		reset();
	}

	public final void setTrajectory(final TrajectoryIterator<TimedState<RigidTransformWithCurvature>> trajectory) {
		m_trajectory = trajectory;
		m_setpoint = trajectory.getState();

		for (int i = 0; i < trajectory.getTrajectory().length(); i++) {
			final var state = trajectory.getTrajectory().getState(i);

			if (state.getVelocity() > Epsilon) {
				m_isReversed = false;
				break;
			} else if (state.getVelocity() < Epsilon) {
				m_isReversed = true;
				break;
			}
		}
	}

	public final void reset() {
		m_error = RigidTransform.identity();
		m_output = new Output();
		m_lastTime = Double.POSITIVE_INFINITY;
		m_previousVelocity = new ChassisState();
	}

	public Optional<Output> update(final double timestamp, final RigidTransform currentPose, final boolean isHighGear) {
		if (Objects.isNull(m_trajectory)) {
			return Optional.empty();
		}

		if (m_trajectory.getProgress() == 0.0 && isInfinite(m_lastTime)) {
			m_lastTime = timestamp;
		}

		final double dt = timestamp - m_lastTime;
		m_lastTime = timestamp;

		final var samplePoint = m_trajectory.advance(dt);
		m_setpoint = samplePoint.getState();

		if (!m_trajectory.isDone()) {
			final var velocityMetersPerSecond = m_setpoint.getVelocity();
			final var curvatureRadiansPerMeter = m_setpoint.getState().getCurvature();
			final var curvatureDsRadiansPerMeterSquared = m_setpoint.getState().getDCurvatureDs();
			final var accelerationMetersPerSecondSquared = m_setpoint.getAcceleration();

			final var setpointDynamics = m_model.solveInverseDynamics(
					new ChassisState(velocityMetersPerSecond, velocityMetersPerSecond * curvatureRadiansPerMeter),
					new ChassisState(accelerationMetersPerSecondSquared,
                            accelerationMetersPerSecondSquared * curvatureRadiansPerMeter
                                    + velocityMetersPerSecond * velocityMetersPerSecond * curvatureDsRadiansPerMeterSquared),
                    isHighGear
			);

			m_error = currentPose.inverse().transformBy(m_setpoint.getState().getRigidTransform());

			switch (m_followerType) {
                case FEEDFORWARD:
                    m_output.velocity = setpointDynamics.wheelVelocity;
                    m_output.acceleration = setpointDynamics.wheelAcceleration;
                    m_output.feedforwardVoltage = setpointDynamics.voltage;
                    break;
				case RAMSETE:
					m_output = updateRamsete(dt, setpointDynamics, currentPose, isHighGear);
					break;
			}
		} else {
			return Optional.empty();
		}

		return Optional.of(m_output);
	}

	// Implements eqn. 5.12 from https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf
	protected Output updateRamsete(final double dt, final Model.Dynamics dynmx, final RigidTransform currentPose, final boolean isHighGear) {
		final double k = 2.0 * m_config.zeta * sqrt(m_config.beta * dynmx.chassisVelocity.linearMeters * dynmx.chassisVelocity.linearMeters + dynmx.chassisVelocity.angularRadians * dynmx.chassisVelocity.angularRadians);

		final var angularErrorRadians = m_error.getRotation().getRadians();
		final var sinXOverX = epsilonEquals(angularErrorRadians, 0.0, 0.01) ? 1.0 : m_error.getRotation().sin() / angularErrorRadians;

		final var adjustedVelocity = new ChassisState(
				dynmx.chassisVelocity.linearMeters * m_error.getRotation().cos()
                        + k * m_error.getTranslation().x(),
				dynmx.chassisVelocity.angularRadians
					    + k * angularErrorRadians
					    + dynmx.chassisVelocity.linearMeters * m_config.beta * sinXOverX * m_error.getTranslation().y());

		dynmx.chassisVelocity = adjustedVelocity;
		// this is where everything goes from meters to wheel radians/s!!
		dynmx.wheelVelocity = m_model.inverseKinematics(adjustedVelocity);

		if (dt == 0.0) {
			dynmx.chassisAcceleration.linearMeters = 0.0;
			dynmx.chassisAcceleration.angularRadians = 0.0;
		} else {
			dynmx.chassisAcceleration.linearMeters = (dynmx.chassisVelocity.linearMeters - m_previousVelocity.linearMeters) / dt;
			dynmx.chassisAcceleration.angularRadians = (dynmx.chassisVelocity.angularRadians - m_previousVelocity.angularRadians) / dt;
		}

		// store previous velocity, allows the user to only have to worry about passing the new state
        // this is superior to passing velocity and acceleration in, like 1678 does, since it allows the user to worry about fewer calculations up front
        // and works fine with a variant dt. However, where it lacks is in application-
        // our Ramsete controller is highly coupled. This should be fine. - Ethan, 9/24/19
		m_previousVelocity = dynmx.chassisVelocity;

		final var output = new Output();
		output.velocity = dynmx.wheelVelocity;
		output.acceleration = dynmx.wheelAcceleration;
		output.feedforwardVoltage = m_model.solveInverseDynamics(dynmx.chassisVelocity, dynmx.chassisAcceleration, isHighGear).voltage;
		return output;
	}

    public final TimedState<RigidTransformWithCurvature> getSetpoint() {
        return m_setpoint;
    }

	public final RigidTransform getError() {
		return m_error;
	}

    @Log.Graph(name="Pose Error (m)", visibleTime=15.0)
    protected double getErrorMagnitude() {
        return m_error.getTranslation().norm();
    }

    @Log.Graph(name="Pose Error (deg)", visibleTime=15.0)
    protected double getErrorHeading() {
        return abs(m_error.getRotation().getDegrees());
    }

	@Log.BooleanBox(name="Complete? (Boolean)")
	public final boolean isDone() {
		return !Objects.isNull(m_trajectory) && m_trajectory.isDone();
	}
}
