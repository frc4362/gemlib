package com.gemsrobotics.lib.controls;

import com.gemsrobotics.lib.subsystems.drivetrain.ChassisState;
import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDriveModel;
import com.gemsrobotics.lib.subsystems.drivetrain.WheelState;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.RigidTransformWithCurvature;
import com.gemsrobotics.lib.trajectory.TrajectoryIterator;
import com.gemsrobotics.lib.trajectory.parameterization.*;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import java.util.Objects;
import java.util.Optional;

import static com.gemsrobotics.lib.utils.MathUtils.*;
import static java.lang.Double.isInfinite;
import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

// consider refactoring into a strategy-pattern with a PathController interface
public class DriveMotionPlanner implements Loggable {
    @Override
    public String configureLogName() {
        return "Motion Planner";
    }


	protected transient final DifferentialDriveModel m_model;
	protected transient final MotionPlanner.MotionConfig m_config;

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
	@Log.ToString(name="Error (Pose)")
	protected RigidTransform m_error;
	@Log.ToString(name="Setpoint (Timed Pose w/ Curvature)")
	protected TimedState<RigidTransformWithCurvature> m_setpoint;
    @Log(name="Reversed? (Boolean)")
	protected boolean m_isReversed;
	protected double m_lastTime;
	protected MotionPlanner.Output m_output;
	protected ChassisState m_previousVelocity;

	public DriveMotionPlanner(
			final MotionPlanner.MotionConfig config,
			final DifferentialDriveModel model,
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
		m_output = new MotionPlanner.Output();
		m_lastTime = Double.POSITIVE_INFINITY;
		m_previousVelocity = new ChassisState();
	}

	public Optional<MotionPlanner.Output> update(final double timestamp, final RigidTransform currentPose, final boolean isHighGear) {
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
                    m_output.velocityRadiansPerSecond = setpointDynamics.wheelVelocityRadiansPerSecond;
                    m_output.accelerationRadiansPerSecondSquared = setpointDynamics.wheelAccelerationRadiansPerSecondSquared;
                    m_output.feedforwardVoltage = setpointDynamics.voltage;
                    break;
				case RAMSETE:
					m_output = updateRamsete(dt, setpointDynamics, isHighGear);
					break;
			}
		} else {
			return Optional.empty();
		}

		return Optional.of(m_output);
	}

	// Implements eqn. 5.12 from https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf
	protected MotionPlanner.Output updateRamsete(final double dt, final DifferentialDriveModel.Dynamics ref, final boolean isHighGear) {
		final double k = 2.0 * 0.7 * sqrt(2.0 * ref.chassisVelocity.linear * ref.chassisVelocity.linear + ref.chassisVelocity.angular * ref.chassisVelocity.angular);

		final var angularErrorRadians = m_error.getRotation().getRadians();

		// adjust for error
		ref.chassisVelocity = new ChassisState(
				ref.chassisVelocity.linear * m_error.getRotation().cos()
						+ k * m_error.getTranslation().x(),
				ref.chassisVelocity.angular
						+ k * angularErrorRadians
						+ ref.chassisVelocity.linear * 2.0 * sinc(angularErrorRadians, 0.01) * m_error.getTranslation().y());
		// this is where everything goes from meters to wheel radians/s!!
		ref.wheelVelocityRadiansPerSecond = m_model.inverseKinematics(ref.chassisVelocity);

		if (dt == 0.0) {
			ref.chassisAcceleration.linear = 0.0;
			ref.chassisAcceleration.angular = 0.0;
		} else {
			ref.chassisAcceleration.linear = (ref.chassisVelocity.linear - m_previousVelocity.linear) / dt;
			ref.chassisAcceleration.angular = (ref.chassisVelocity.angular - m_previousVelocity.angular) / dt;
		}

		// store previous velocity, allows the user to only have to worry about passing the new state
        // this is superior to passing velocity and acceleration in, like 1678 does, since it allows the user to worry
		// about fewer calculations up front and works fine with a variant dt. However, where it lacks is in application-
        // our Ramsete controller is highly coupled. This should be fine. - Ethan, 9/24/19
		m_previousVelocity = ref.chassisVelocity;

		final var output = new MotionPlanner.Output();
		output.velocityRadiansPerSecond = ref.wheelVelocityRadiansPerSecond;
		output.accelerationRadiansPerSecondSquared = ref.wheelAccelerationRadiansPerSecondSquared;
		output.feedforwardVoltage = m_model.solveInverseDynamics(ref.chassisVelocity, ref.chassisAcceleration, isHighGear).voltage;
		return output;
	}

    public final TimedState<RigidTransformWithCurvature> getReference() {
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
