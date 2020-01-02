package lib.trajectory;

import com.gemsrobotics.lib.controls.DriveMotionPlanner;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.RigidTransformWithCurvature;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.physics.MotorModel;
import com.gemsrobotics.lib.subsystems.drivetrain.ChassisState;
import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDriveModel;
import com.gemsrobotics.lib.trajectory.*;
import com.gemsrobotics.lib.trajectory.parameterization.DifferentialDriveDynamicsConstraint;
import com.gemsrobotics.lib.trajectory.parameterization.Parameterizer;
import com.gemsrobotics.lib.trajectory.parameterization.TimedState;
import com.gemsrobotics.lib.trajectory.parameterization.TrajectoryUtils;
import org.junit.Test;

import static org.hamcrest.MatcherAssert.assertThat;
import static org.hamcrest.Matchers.*;
import static org.junit.Assert.assertFalse;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class TestIntegration {
    @Test
    public void testSplineTrajectoryGenerator() {
        // Specify desired waypoints.
        final List<RigidTransform> waypoints = Arrays.asList(
                RigidTransform.identity(),
                new RigidTransform(100, 0, Rotation.degrees(0)));

        final var cfg = new DriveMotionPlanner.MotionConfig() {
            {
                maxDx = 0.0508;
                maxDy = 0.00127;
                maxDtheta = Math.toRadians(5.0);
                maxVoltage = 12.0;
                maxVelocity = 4.0;
                maxAcceleration = 3.0;
                maxCentripetalAcceleration = 1.0;
            }
        };

        // Create a trajectory from splines.
        Trajectory<RigidTransformWithCurvature> trajectory = TrajectoryUtils.trajectoryFromSplineWaypoints(waypoints, cfg);

        final var modelProps = new DifferentialDriveModel.Properties() {
            {
                massKg = 60;
                angularMomentInertiaKgMetersSquared = 9.6;
                angularDragTorquePerRadiansPerSecond = 12.0;
                wheelRadiusMeters = 0.0508;
                wheelbaseRadiusMeters = 0.3489513;
            }
        };

        final var transmission = new MotorModel(new MotorModel.Properties() {{
            speedRadiansPerSecondPerVolt = 7.11942257233;
            torquePerVolt = 6.4516;
            stictionVoltage = 1.3;
        }});

        final var model = new DifferentialDriveModel(modelProps, transmission, transmission);

        // Create the constraint that the robot must be able to traverse the trajectory without ever applying more than 10V.
        DifferentialDriveDynamicsConstraint<RigidTransformWithCurvature> constraint = new DifferentialDriveDynamicsConstraint<>(model, false, 10.0);

        // Generate the timed trajectory.
        Trajectory<TimedState<RigidTransformWithCurvature>> timedTrajectory = Parameterizer.timeParameterizeTrajectory(
                false,
                new DistanceView<>(trajectory),
                1.0,
                Collections.singletonList(constraint),
                cfg,
                0.0,
                0.0);

        for (int i = 1; i < timedTrajectory.length(); ++i) {
            TrajectoryPoint<TimedState<RigidTransformWithCurvature>> prev = timedTrajectory.getPoint(i - 1);
            TrajectoryPoint<TimedState<RigidTransformWithCurvature>> next = timedTrajectory.getPoint(i);
            assertThat(prev.state().getAcceleration(), closeTo((next.state().getVelocity() - prev.state().getVelocity()) / (next.state().t() - prev.state().t()), 1e-9));
            final double dt = next.state().t() - prev.state().t();
            assertThat(next.state().getVelocity(), closeTo(prev.state().getVelocity() + prev.state().getAcceleration() * dt, 1E-9));
            assertThat(next.state().distance(prev.state()), closeTo(prev.state().getVelocity() * dt + 0.5 * prev.state().getAcceleration() * dt * dt, 1E-9));
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

            System.out.println(dynamics.voltage.map(v -> v / 12.0));

            sample = iterator.advance(dt);
        } while (!iterator.isDone());
    }
}
