package lib.trajectory;

import com.gemsrobotics.lib.controls.DriveMotionPlanner;
import com.gemsrobotics.lib.controls.MotionPlanner;
import com.gemsrobotics.lib.controls.PIDFController;
import com.gemsrobotics.lib.math.se2.*;
import com.gemsrobotics.lib.physics.MotorModel;
import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDrive;
import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDriveModel;
import com.gemsrobotics.lib.subsystems.drivetrain.OpenLoopDriveHelper;
import com.gemsrobotics.lib.trajectory.MirroredTrajectory;
import com.gemsrobotics.lib.trajectory.TimedView;
import com.gemsrobotics.lib.trajectory.TrajectoryIterator;
import com.gemsrobotics.lib.trajectory.TrajectoryPoint;
import com.gemsrobotics.lib.trajectory.parameterization.TimedState;
import com.gemsrobotics.lib.trajectory.parameterization.TrajectoryGenerator;
import com.gemsrobotics.lib.utils.FastDoubleToString;
import com.gemsrobotics.lib.utils.Units;
import com.google.gson.Gson;
import org.junit.Test;

import java.io.IOException;
import java.lang.reflect.Field;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

import static com.gemsrobotics.lib.utils.MathUtils.epsilonEquals;
import static com.gemsrobotics.lib.utils.MathUtils.Epsilon;
import static java.lang.Math.abs;
import static org.hamcrest.MatcherAssert.assertThat;
import static org.hamcrest.Matchers.*;

public class TestTrajectoryGeneration {
    private DifferentialDrive.Config cfg;
    private DifferentialDriveModel model;
    private TrajectoryGenerator generator;

    public void verifyValidPose(final RigidTransform pose) {
        assertThat(pose.getRotation().getDegrees(), lessThanOrEqualTo(180.0));
        assertThat(pose.getRotation().getDegrees(), greaterThanOrEqualTo(-180.0));
    }

    public void reflectingPoseValidator(final TrajectoryGenerator trajectoryGenerator) {
        Field[] fields = TrajectoryGenerator.class.getFields();
        Arrays.sort(fields, Comparator.comparing(Field::getName));
        for (final Field f: fields) {
            try {
                final var p = (RigidTransform) f.get(trajectoryGenerator);
                verifyValidPose(p);
            } catch (Exception ex) {

            }
        }
    }

    @Test
    public void test() {
        final double kS = 0.36167; // V
        final double kV = 0.1329; // V / (rad / s)
        final double kA = 0.012; // V / (rad / s^2)

        final double wheelRadius = 0.08016875;
        final double mass = 62.73; // kg

        final var cfg = new DifferentialDrive.Config() {
            {
                maxVoltage = 12.0;
                secondsToMaxVoltage = 0.1;

                motionConfig = new MotionPlanner.MotionConfig() {{
                    maxDx = 0.00127;
                    maxDy = 0.00127;
                    maxDtheta = Rotation.degrees(5.0).getRadians();
                    maxVoltage = 10.0;
                    maxVelocity = 4.8;
                    maxAcceleration = 3.2;
                    maxCentripetalAcceleration = 2.0;
                }};

                gainsLowGear = new PIDFController.Gains(5.33, 0.0, 0.0, 0.0);
                gainsHighGear = gainsLowGear;

                propertiesLowGear = new MotorModel.Properties() {{
                    speedRadiansPerSecondPerVolt = (1 / kV);
                    torquePerVolt = wheelRadius * wheelRadius * mass / (2.0 * kA);
                    stictionVoltage = kS;
                }};
                propertiesHighGear = propertiesLowGear;

                propertiesModel = new DifferentialDriveModel.Properties() {
                    {
                        massKg = 62.73;
                        angularMomentInertiaKgMetersSquared = Math.pow(Units.inches2Meters(6.0), 2) * massKg;
                        angularDragTorquePerRadiansPerSecond = 12.0;
                        wheelRadiusMeters = 0.08016875;
                        wheelbaseRadiusMeters = 0.351;
                    }
                };

                openLoopConfig = new OpenLoopDriveHelper.Config() {
                    {
                        quickTurnScalar = 1.0;
                    }
                };
            }
        };

        final var transmission = new MotorModel(new MotorModel.Properties() {{
            speedRadiansPerSecondPerVolt = (1 / kV);
            torquePerVolt = wheelRadius * wheelRadius * mass / (2.0 * kA);
            stictionVoltage = kS;
        }});

        final var props = new DifferentialDriveModel.Properties() {{
            massKg = mass; // kg
            angularMomentInertiaKgMetersSquared = Math.pow(Units.inches2Meters(6.0), 2) * massKg;
            angularDragTorquePerRadiansPerSecond = 12.0;
            wheelRadiusMeters = wheelRadius;
            wheelbaseRadiusMeters = 0.351;
        }};

        model = new DifferentialDriveModel(props, transmission);
        generator = new TrajectoryGenerator(cfg.motionConfig, model);

        reflectingPoseValidator(generator);

        final var reversed = false;

        final var trajectory = generator.generateTrajectory(
                false,
                reversed,
                List.of(
                        new RigidTransform(Translation.identity(), Rotation.identity()),
                        new RigidTransform(new Translation(5.0, -3.5), Rotation.degrees(-90))
                ),
                Collections.emptyList(),
                0.0,
                0.0);

//        final var trajectory = generator.generateTrajectory(
//                false,
//                reversed,
//                Arrays.asList(
//                        RigidTransform.identity(),
//                        new RigidTransform(100 * 0.0254, 0, Rotation.degrees(0)),
//                        new RigidTransform(140, 36, Rotation.degrees(0))),
//                Collections.emptyList(),
//                0.0,
//                0.0);

//        System.out.println(trajectory.getTrajectory().getTrajectory().getPoints().stream()
//              .map(TrajectoryPoint::state)
//              .map(state -> {
//                  final var trans = state.getState().getTranslation();
//                  return FastDoubleToString.format(trans.x()) + ", "
//                         + FastDoubleToString.format(trans.y())
//                         + ", " + FastDoubleToString.format(state.getState().getRotation().getDegrees());
//              })
//              .collect(Collectors.joining("\n")));

        verifyMirroredTrajectories(new MirroredTrajectory(trajectory.getTrajectory().getTrajectory()), reversed);
    }

    public void verifyMirroredTrajectories(final MirroredTrajectory mirrored, final boolean shouldBeReversed) {
        assertThat(mirrored.left.length(), equalTo(mirrored.right.length()));
        final var iteratorLeft = new TrajectoryIterator<>(new TimedView<>(mirrored.left));
        final var iteratorRight = new TrajectoryIterator<>(new TimedView<>(mirrored.right));

        final double dt = 0.05;
        RigidTransformWithCurvature previousLeftState = null;
        RigidTransformWithCurvature previousRightState = null;

        while (!iteratorLeft.isDone() && !iteratorRight.isDone()) {
            TimedState<RigidTransformWithCurvature> stateLeft = iteratorLeft.getState();
            TimedState<RigidTransformWithCurvature> stateRight = iteratorRight.getState();

            assertThat(stateLeft.t(), closeTo(stateRight.t(), Epsilon));
            assertThat(stateLeft.getVelocity(), closeTo(stateRight.getVelocity(), Epsilon));
            assertThat(stateLeft.getAcceleration(), closeTo(stateRight.getAcceleration(), Epsilon));

            assertThat((shouldBeReversed ? -1.0 : 1.0) * stateLeft.getVelocity(), greaterThanOrEqualTo(-Epsilon));
            assertThat((shouldBeReversed ? -1.0 : 1.0) * stateRight.getVelocity(), greaterThanOrEqualTo(-Epsilon));

            final RigidTransformWithCurvature poseLeft = stateLeft.getState();
            final RigidTransformWithCurvature poseRight = stateRight.getState();

            if (previousLeftState != null && previousRightState != null) {
                // Check there are no angle discontinuities.
                final double maxReasonableChangeInAngle = 0.3;  // rad

                Twist deltaLeft = previousLeftState.getRigidTransform().inverse().transformBy(poseLeft.getRigidTransform()).toTwist();
                Twist deltaRight = previousRightState.getRigidTransform().inverse().transformBy(poseRight.getRigidTransform()).toTwist();

                assertThat(abs(deltaLeft.dtheta), lessThan(maxReasonableChangeInAngle));
                assertThat(abs(deltaRight.dtheta), lessThan(maxReasonableChangeInAngle));

                if (!epsilonEquals(deltaLeft.dtheta, 0.0) || !epsilonEquals(deltaRight.dtheta, 0.0)) {
                    // Could be a curvature sign change between prev and now, so just check that either matches our expected sign.
                    final boolean isLeftCurvaturePositive = poseLeft.getCurvature() > Epsilon || previousLeftState.getCurvature() > Epsilon;
                    final boolean isLeftCurvatureNegative = poseLeft.getCurvature() < -Epsilon || previousLeftState.getCurvature() < -Epsilon;
                    final boolean isRightCurvaturePositive = poseRight.getCurvature() > Epsilon || previousRightState.getCurvature() > Epsilon;
                    final boolean isRightCurvatureNegative = poseRight.getCurvature() < -Epsilon || previousRightState.getCurvature() < -Epsilon;

                    final double curvatureLeft = deltaLeft.dtheta / deltaLeft.dx;
                    final double curvatureRight = deltaRight.dtheta / deltaRight.dx;

                    if (curvatureLeft < -Epsilon) {
                        assertThat(isLeftCurvatureNegative, is(true));
                    } else if (curvatureLeft > Epsilon) {
                        assertThat(isLeftCurvaturePositive, is(true));
                    }

                    if (curvatureRight < -Epsilon) {
                        assertThat(isRightCurvatureNegative, is(true));
                    } else if (curvatureRight > Epsilon) {
                        assertThat(isRightCurvaturePositive, is(true));
                    }
                }
            }

            assertThat(poseLeft.getTranslation().x(), closeTo(poseRight.getTranslation().x(), Epsilon));
            assertThat(poseLeft.getTranslation().y(), closeTo(-poseRight.getTranslation().y(), Epsilon));
            assertThat(poseLeft.getRotation().getRadians(), closeTo(poseRight.getRotation().inverse().getRadians(), Epsilon));
            assertThat(poseLeft.getCurvature(), closeTo(-poseRight.getCurvature(), Epsilon));

            iteratorLeft.advance(dt);
            iteratorRight.advance(dt);

            previousLeftState = poseLeft;
            previousRightState = poseRight;
        }

        assertThat(iteratorLeft.isDone() && iteratorRight.isDone(), is(true));
    }
}
