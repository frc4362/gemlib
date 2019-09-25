package lib;

import com.gemsrobotics.lib.math.se2.*;
import com.gemsrobotics.lib.physics.MotorTransmission;
import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDrive;
import com.gemsrobotics.lib.subsystems.drivetrain.Model;
import com.gemsrobotics.lib.trajectory.MirroredTrajectory;
import com.gemsrobotics.lib.trajectory.TimedView;
import com.gemsrobotics.lib.trajectory.TrajectoryIterator;
import com.gemsrobotics.lib.trajectory.parameterization.TimedState;
import com.gemsrobotics.lib.trajectory.parameterization.TrajectoryGenerator;
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
import static com.gemsrobotics.lib.utils.MathUtils.kEpsilon;
import static java.lang.Math.abs;
import static org.hamcrest.MatcherAssert.assertThat;
import static org.hamcrest.Matchers.*;

public class TestTrajectoryGeneration {
    private DifferentialDrive.Config cfg;
    private Model model;
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
        try {
            final var raw = String.join("", Files.readAllLines(Path.of("E:\\gemlib\\src\\main\\deploy\\drivetrain_properties.json")));
            cfg = new Gson().fromJson(raw, DifferentialDrive.Config.class);
        } catch (final IOException e) {
            e.printStackTrace();
        }

        final var transmission = new MotorTransmission(Units.rpm2RadsPerSecond(65.0), 0.35, 1.0);
        final var props = new Model.Properties() {
            {
                mass = 63;
                momentInertia = 84;
                angularDrag = 12.0;
                wheelRadius = Units.inches2Meters(2.0);
                wheelbaseRadius = Units.inches2Meters(25.0) / 2.0;
            }
        };

        model = new Model(props, transmission);
        generator = new TrajectoryGenerator(cfg.motionConfig, model);

        reflectingPoseValidator(generator);

        final var reversed = true;

        final var trajectory = generator.generateTrajectory(
                false,
                reversed,
                Arrays.asList(
                        RigidTransform.identity(),
                        new RigidTransform(-100 * 0.0254, -100 * 0.0254, Rotation.degrees(90))),
                Collections.emptyList(),
                0.0,
                0.0);

        System.out.println(trajectory.trajectory.getPoints().stream().map(point -> point.state().toString()).collect(Collectors.joining("\n")));

        verifyMirroredTrajectories(new MirroredTrajectory(trajectory.trajectory), reversed);
    }

    public void verifyMirroredTrajectories(final MirroredTrajectory mirrored, final boolean shouldBeReversed) {
        assertThat(mirrored.left.length(), equalTo(mirrored.right.length()));
        final var iteratorLeft = new TrajectoryIterator<>(new TimedView<>(mirrored.left));
        final var iteratorRight = new TrajectoryIterator<>(new TimedView<>(mirrored.right));

        final double dt = 0.05;
        TimedState<RigidTransformWithCurvature> previousLeftState = null;
        TimedState<RigidTransformWithCurvature> previousRightState = null;

        while (!iteratorLeft.isDone() && !iteratorRight.isDone()) {
            TimedState<RigidTransformWithCurvature> stateLeft = iteratorLeft.getState();
            TimedState<RigidTransformWithCurvature> stateRight = iteratorRight.getState();

            assertThat(stateLeft.t(), closeTo(stateRight.t(), kEpsilon));
            assertThat(stateLeft.getVelocity(), closeTo(stateRight.getVelocity(), kEpsilon));
            assertThat(stateLeft.getAcceleration(), closeTo(stateRight.getAcceleration(), kEpsilon));

            assertThat((shouldBeReversed ? -1.0 : 1.0) * stateLeft.getVelocity(), greaterThanOrEqualTo(-kEpsilon));
            assertThat((shouldBeReversed ? -1.0 : 1.0) * stateRight.getVelocity(), greaterThanOrEqualTo(-kEpsilon));

            if (previousLeftState != null && previousRightState != null) {
                // Check there are no angle discontinuities.
                final double kMaxReasonableChangeInAngle = 0.3;  // rad

                Twist deltaLeft = RigidTransform.toTwist(previousLeftState.getState().getRigidTransform().inverse().transformBy(stateLeft.getState().getRigidTransform()));
                Twist deltaRight = RigidTransform.toTwist(previousRightState.getState().getRigidTransform().inverse().transformBy(stateRight.getState().getRigidTransform()));

                assertThat(abs(deltaLeft.dtheta), lessThan(kMaxReasonableChangeInAngle));
                assertThat(abs(deltaRight.dtheta), lessThan(kMaxReasonableChangeInAngle));

                if (!epsilonEquals(deltaLeft.dtheta, 0.0) || !epsilonEquals(deltaRight.dtheta, 0.0)) {
                    // Could be a curvature sign change between prev and now, so just check that either matches our expected sign.
                    final boolean isLeftCurvaturePositive = stateLeft.getState().getCurvature() > kEpsilon || previousLeftState.getState().getCurvature() > kEpsilon;
                    final boolean isLeftCurvatureNegative = stateLeft.getState().getCurvature() < -kEpsilon || previousLeftState.getState().getCurvature() < -kEpsilon;
                    final boolean isRightCurvaturePositive = stateRight.getState().getCurvature() > kEpsilon || previousRightState.getState().getCurvature() > kEpsilon;
                    final boolean isRightCurvatureNegative = stateRight.getState().getCurvature() < -kEpsilon || previousRightState.getState().getCurvature() < -kEpsilon;

                    final double curvatureLeft = deltaLeft.dtheta / deltaLeft.dx;
                    final double curvatureRight = deltaRight.dtheta / deltaRight.dx;

                    if (curvatureLeft < -kEpsilon) {
                        assertThat(isLeftCurvatureNegative, is(true));
                    } else if (curvatureLeft > kEpsilon) {
                        assertThat(isLeftCurvaturePositive, is(true));
                    }

                    if (curvatureRight < -kEpsilon) {
                        assertThat(isRightCurvatureNegative, is(true));
                    } else if (curvatureRight > kEpsilon) {
                        assertThat(isRightCurvaturePositive, is(true));
                    }
                }
            }

            assertThat(stateLeft.getState().getTranslation().x(), closeTo(stateRight.getState().getTranslation().x(), kEpsilon));
            assertThat(stateLeft.getState().getTranslation().y(), closeTo(-stateRight.getState().getTranslation().y(), kEpsilon));
            assertThat(stateLeft.getState().getRotation().getRadians(), closeTo(stateRight.getState().getRotation().inverse().getRadians(), kEpsilon));
            assertThat(stateLeft.getState().getCurvature(), closeTo(-stateRight.getState().getCurvature(), kEpsilon));

            iteratorLeft.advance(dt);
            iteratorRight.advance(dt);
            previousLeftState = stateLeft;
            previousRightState = stateRight;
        }

        assertThat(iteratorLeft.isDone() && iteratorRight.isDone(), is(true));
    }
}
