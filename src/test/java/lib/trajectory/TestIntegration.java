package lib.trajectory;

import com.gemsrobotics.lib.controls.DriveMotionPlanner;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.RigidTransformWithCurvature;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.physics.MotorTransmission;
import com.gemsrobotics.lib.subsystems.drivetrain.ChassisState;
import com.gemsrobotics.lib.subsystems.drivetrain.Model;
import com.gemsrobotics.lib.trajectory.*;
import com.gemsrobotics.lib.trajectory.parameterization.Parameterizer;
import com.gemsrobotics.lib.trajectory.parameterization.TimedState;
import com.gemsrobotics.lib.trajectory.parameterization.TrajectoryUtils;
import com.gemsrobotics.lib.utils.Units;
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
        List<RigidTransform> waypoints = Arrays.asList(
                new RigidTransform(0.0, 0.0, Rotation.degrees(0.0)),
                new RigidTransform(36.0, 0.0, Rotation.degrees(0.0)),
                new RigidTransform(60.0, 100, Rotation.degrees(0.0)),
                new RigidTransform(160.0, 100.0, Rotation.degrees(0.0)),
                new RigidTransform(200.0, 70, Rotation.degrees(45.0)));

        final var cfg = new DriveMotionPlanner.MotionConfig() {
            {
                maxDx = 2.0;
                maxDy = 0.2;
                maxDtheta = Math.toRadians(5.0);
                maxVoltage = 12.0;
                maxVelocity = 12 * 14;
                maxAcceleration = 12 * 10;
                maxCentripetalAcceleration = 15 * 12;
            }
        };

        // Create a trajectory from splines.
        Trajectory<RigidTransformWithCurvature> trajectory = TrajectoryUtils.trajectoryFromSplineWaypoints(waypoints, cfg);

        final var modelProps = new Model.Properties() {
            {
                massKg = 60.0;
                momentInertiaKgMetersSquared = 80.0;
                angularDragTorquePerRadiansPerSecond = 0.0;
                wheelRadiusMeters = Units.inches2Meters(2.0);
                wheelbaseRadiusMeters = Units.inches2Meters(26.0 / 2.0);
            }
        };

        final var transmission = new MotorTransmission(
                1.0 / 0.143,
                (modelProps.wheelRadiusMeters * modelProps.wheelRadiusMeters * modelProps.massKg / 2.0) / 0.02,
                0.8);

        final var model = new Model(modelProps, transmission, transmission);

        // Create the constraint that the robot must be able to traverse the trajectory without ever applying more than 10V.
//        DifferentialDriveDynamicsConstraint<RigidTransformWithCurvature> constraints = new DifferentialDriveDynamicsConstraint<>(model, false, 10.0);

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
            assertThat(prev.state().getAcceleration(), closeTo((next.state().getVelocity() - prev.state().getVelocity()) / (next.state().t() - prev.state().t()), 1e-9));
            final double dt = next.state().t() - prev.state().t();
            assertThat(next.state().getVelocity(), closeTo(prev.state().getVelocity() + prev.state().getAcceleration() * dt, 1E-9));
            assertThat(next.state().distance(prev.state()), closeTo(prev.state().getVelocity() * dt + 0.5 * prev.state().getAcceleration() * dt * dt, 1E-9));
        }

        // "Follow" the trajectory.
        final double dt = 0.01;
        boolean first = true;

        TrajectoryIterator<TimedState<RigidTransformWithCurvature>> iterator = new TrajectoryIterator<>(new TimedView<>(timedTrajectory));

        while (!iterator.isDone()) {
            TrajectorySamplePoint<TimedState<RigidTransformWithCurvature>> sample;

            if (first) {
                sample = iterator.getSample();
                first = false;
            } else {
                sample = iterator.advance(dt);
            }

            final TimedState<RigidTransformWithCurvature> state = sample.getState();

            final Model.Dynamics dynamics = model.solveInverseDynamics(
                    new ChassisState(Units.inches2Meters(state.getVelocity()), state.getVelocity() * state.getState().getCurvature()),
                    new ChassisState(Units.inches2Meters(state.getAcceleration()), state.getAcceleration() * state.getState().getCurvature()),
                    false);
        }
    }
}
