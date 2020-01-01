package lib.trajectory;

import com.gemsrobotics.lib.controls.DriveMotionPlanner;
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
        final var cfg = new DifferentialDrive.Config() {
            {
                rotationsToMeters = 0.00072;
                maxVoltage = 12.0;
                secondsToMaxVoltage = 0.2;

                motionConfig = new DriveMotionPlanner.MotionConfig() {
                    {
                        beta = 2.0;
                        zeta = 0.7;
                        maxDx = 0.0508;
                        maxDy = 0.00127;
                        maxDtheta = 0.1;
                        maxVoltage = 10.0;
                        maxVelocity = 3.96;
                        maxAcceleration = 5.0;
                        maxCentripetalAcceleration = 1.57;
                    }
                };

                gainsLowGear = new PIDFController.Gains(0.002, 0.0, 0.0, 0.0225);
                gainsHighGear = gainsLowGear;

                propertiesLowGear = new MotorModel.Properties() {
                    {
                        stictionVoltage = 0.3017;
                        speedRadiansPerSecondPerVolt = 0.1862;
                        torquePerVolt = 0.0086739979;
                    }
                };
                propertiesHighGear = propertiesLowGear;

                propertiesModel = new DifferentialDriveModel.Properties() {
                    {
                        massKg = 63.5;
                        angularMomentInertiaKgMetersSquared = 4.0;
                        angularDragTorquePerRadiansPerSecond = 1.2;
                        wheelRadiusMeters = 0.0482;
                        wheelbaseRadiusMeters = 0.8763;
                    }
                };

                openLoopConfig = new OpenLoopDriveHelper.Config() {
                    {
                        useSineAttack = true;
                        zNonLinearityHighGear = 1.0;
                        zNonLinearityLowGear = 1.0;

                        sensitivityHighGear = 0.65;
                        sensitivityLowGear = 1.0;

                        negativeInertiaScalarHigh = 4.0;
                        negativeInertiaThresholdLow = 0.65;
                        negativeInertiaTurnScalarLow = 3.5;
                        negativeInertiaCloseScalarLow = 4.0;
                        negativeInertiaFarScalarLow = 5.0;

                        quickStopDeadband = 0.5;
                        quickStopScalar = 5.0;
                        quickStopWeight = 0.1;
                    }
                };
            }
        };

        final var transmission = new MotorModel(new MotorModel.Properties() {{
            speedRadiansPerSecondPerVolt = Units.rpm2RadsPerSecond(65.0);
            torquePerVolt = 0.35;
            stictionVoltage = 1.0;
        }});

        final var props = new DifferentialDriveModel.Properties() {
            {
                massKg = 63;
                angularMomentInertiaKgMetersSquared = 84;
                angularDragTorquePerRadiansPerSecond = 12.0;
                wheelRadiusMeters = Units.inches2Meters(2.0);
                wheelbaseRadiusMeters = Units.inches2Meters(25.0) / 2.0;
            }
        };

        model = new DifferentialDriveModel(props, transmission);
        generator = new TrajectoryGenerator(cfg.motionConfig, model);

        reflectingPoseValidator(generator);

        final var reversed = false;

        final var trajectory = generator.generateTrajectory(
                false,
                reversed,
                Arrays.asList(
                        RigidTransform.identity(),
                        new RigidTransform(100 * 0.0254, 0, Rotation.degrees(0)),
                        new RigidTransform(140, 36, Rotation.degrees(0))),
                Collections.emptyList(),
                0.0,
                0.0);

        System.out.println(trajectory.getTrajectory().getTrajectory().getPoints().stream()
              .map(TrajectoryPoint::state)
              .map(state -> {
                  final var trans = state.getState().getTranslation();
                  return FastDoubleToString.format(trans.x()) + ", "
                         + FastDoubleToString.format(trans.y())
                         + ", " + FastDoubleToString.format(state.getState().getRotation().getDegrees());
              })
              .collect(Collectors.joining("\n")));

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
