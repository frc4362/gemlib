package lib.trajectory;

import com.gemsrobotics.lib.controls.DriveMotionPlanner;
import com.gemsrobotics.lib.controls.MotionPlanner;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.RigidTransformWithCurvature;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.physics.MotorModel;
import com.gemsrobotics.lib.subsystems.drivetrain.ChassisState;
import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDriveModel;
import com.gemsrobotics.lib.subsystems.drivetrain.WheelState;
import com.gemsrobotics.lib.trajectory.*;
import com.gemsrobotics.lib.trajectory.parameterization.DifferentialDriveDynamicsConstraint;
import com.gemsrobotics.lib.trajectory.parameterization.Parameterizer;
import com.gemsrobotics.lib.trajectory.parameterization.TimedState;
import com.gemsrobotics.lib.trajectory.parameterization.TrajectoryUtils;
import com.gemsrobotics.lib.utils.Units;
import org.junit.Test;

import static org.hamcrest.MatcherAssert.assertThat;
import static org.hamcrest.Matchers.*;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class TestIntegration {
    @Test
    public void testSplineTrajectoryGenerator() {
        // Specify desired waypoints.
        final List<RigidTransform> waypoints = Arrays.asList(
                new RigidTransform(0, 0, Rotation.identity()),
				new RigidTransform(4, 0, Rotation.identity()));

        final double peakVoltage = 12.0;
        final double wheelRadius = 0.08016875;
        final double freeSpeed = 4.8 / wheelRadius; // r/s
        final double kS = 0.36167; // V
        final double kV = 0.1329; // V / (rad / s)
        final double kA = 0.012; // V / (rad / s^2)
        final double mass = 62.73; // kg

        final var cfg = new MotionPlanner.MotionConfig() {{
            maxDx = 0.00127;
            maxDy = 0.00127;
            maxDtheta = Rotation.degrees(5.0).getRadians();
            maxVoltage = 10.0;
            maxVelocity = 4.8;
            maxAcceleration = 3.2;
            maxCentripetalAcceleration = 2.0;
        }};

        // Create a trajectory from splines.
        Trajectory<RigidTransformWithCurvature> trajectory = TrajectoryUtils.trajectoryFromSplineWaypoints(waypoints, cfg);

        final var modelProps = new DifferentialDriveModel.Properties() {{
            massKg = mass; // kg
            angularMomentInertiaKgMetersSquared = Math.pow(Units.inches2Meters(6.0), 2) * massKg;
            angularDragTorquePerRadiansPerSecond = 12.0;
            wheelRadiusMeters = wheelRadius;
            wheelbaseRadiusMeters = 0.351;
        }};

        final var transmission = new MotorModel(new MotorModel.Properties() {{
            speedRadiansPerSecondPerVolt = (1 / kV);
            torquePerVolt = wheelRadius * wheelRadius * mass / (2.0 * kA);
            stictionVoltage = kS;
        }});

        final var model = new DifferentialDriveModel(modelProps, transmission, transmission);

        // Generate the timed trajectory.
        Trajectory<TimedState<RigidTransformWithCurvature>> timedTrajectory = Parameterizer.timeParameterizeTrajectory(
                false,
                new DistanceView<>(trajectory),
                1.0,
                Collections.emptyList(),
                cfg,
                0.0,
                0.0);

        for (int i = 1; i < timedTrajectory.length(); ++i) {
            TrajectoryPoint<TimedState<RigidTransformWithCurvature>> prev = timedTrajectory.getPoint(i - 1);
            TrajectoryPoint<TimedState<RigidTransformWithCurvature>> next = timedTrajectory.getPoint(i);
            assertThat(prev.state().getAcceleration(), closeTo((next.state().getVelocity() - prev.state().getVelocity()) / (next.state().t() - prev.state().t()), 1e-6));
            final double dt = next.state().t() - prev.state().t();
            assertThat(next.state().getVelocity(), closeTo(prev.state().getVelocity() + prev.state().getAcceleration() * dt, 1E-6));
            assertThat(next.state().distance(prev.state()), closeTo(prev.state().getVelocity() * dt + 0.5 * prev.state().getAcceleration() * dt * dt, 1E-6));
        }

        // "Follow" the trajectory.
        final double dt = 0.01;

        TrajectoryIterator<TimedState<RigidTransformWithCurvature>> iterator = new TrajectoryIterator<>(new TimedView<>(timedTrajectory));

        var sample = iterator.getSample();

        do {
            final TimedState<RigidTransformWithCurvature> state = sample.getState();

            final DifferentialDriveModel.Dynamics dynamics = model.solveInverseDynamics(
                    new ChassisState(state.getVelocity(), state.getVelocity() * state.getState().getCurvature()),
                    new ChassisState(state.getAcceleration(), state.getAcceleration() * state.getState().getCurvature()),
                    false);

            sample = iterator.advance(dt);
        } while (!iterator.isDone());
    }
}
