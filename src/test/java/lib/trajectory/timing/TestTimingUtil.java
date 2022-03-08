package lib.trajectory.timing;

import com.gemsrobotics.lib.controls.DriveMotionPlanner;
import com.gemsrobotics.lib.controls.MotionPlanner;
import com.gemsrobotics.lib.math.se2.ITranslation2d;
import com.gemsrobotics.lib.math.se2.State;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.trajectory.DistanceView;
import com.gemsrobotics.lib.trajectory.Trajectory;
import com.gemsrobotics.lib.trajectory.parameterization.Parameterizer;
import com.gemsrobotics.lib.trajectory.parameterization.TimedState;
import com.gemsrobotics.lib.trajectory.parameterization.TimingConstraint;
import com.gemsrobotics.lib.trajectory.parameterization.VelocityLimitRegionConstraint;
import org.junit.Test;

import static org.hamcrest.MatcherAssert.assertThat;
import static org.hamcrest.Matchers.*;

import static com.gemsrobotics.lib.utils.MathUtils.Epsilon;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class TestTimingUtil {
    public static final List<Translation> WAYPOINTS = Arrays.asList(
            new Translation(0.0, 0.0),
            new Translation(24.0, 0.0),
            new Translation(36.0, 12.0),
            new Translation(60.0, 12.0));

    private static final MotionPlanner.MotionConfig MOTION_CONFIG = new MotionPlanner.MotionConfig() {
        {
            maxDx = 0.0508; // meters
            maxDy = 0.00127; // meters
            maxDtheta = 0.1; // radians
            maxVoltage = 10.0; // volts
            maxVelocity = 3.96; // meters/second
            maxAcceleration = 5.0; // meters/second^2
            maxCentripetalAcceleration = 1.57; // meters/s
        }
    };

    public <S extends State<S>> Trajectory<TimedState<S>> buildAndCheckTrajectory(
            final DistanceView<S> distanceView,
            final double stepSize,
            final List<TimingConstraint<S>> constraints,
            final MotionPlanner.MotionConfig config,
            final double velocityStart,
            final double velocityEnd
    ) {
        Trajectory<TimedState<S>> timedTrajectory = Parameterizer.timeParameterizeTrajectory(false, distanceView, stepSize, constraints, config, velocityStart, velocityEnd);
        checkTrajectory(timedTrajectory, constraints, config, velocityStart, velocityEnd);
        return timedTrajectory;
    }

    public <S extends State<S>> void checkTrajectory(
            final Trajectory<TimedState<S>> trajectory,
            final List<TimingConstraint<S>> constraints,
            final MotionPlanner.MotionConfig config,
            final double velocityStart,
            final double velocityEnd
    ) {
        assertThat(trajectory.isEmpty(), is(false));
        assertThat(trajectory.getState(0).getVelocity(), closeTo(velocityStart, Epsilon));
        assertThat(trajectory.getState(trajectory.length() - 1).getVelocity(), closeTo(velocityEnd, Epsilon));

        // Go state by state, verifying all constraints are satisfied and integration is correct.
        for (int i = 0; i < trajectory.length(); ++i) {
            final TimedState<S> state = trajectory.getState(i);

            for (final TimingConstraint<S> constraint : constraints) {
                assertThat(state.getVelocity() - Epsilon, lessThanOrEqualTo(constraint.getMaxVelocity(state.getState())));

                final var accelerationBounds = constraint.getMinMaxAcceleration(state.getState(), state.getVelocity());

                assertThat(state.getAcceleration() - Epsilon, lessThanOrEqualTo(accelerationBounds.getAccelerationMax()));
                assertThat(state.getAcceleration() + Epsilon, greaterThanOrEqualTo(accelerationBounds.getAccelerationMin()));
            }

            if (i > 0) {
                final TimedState<S> lastState = trajectory.getState(i - 1);
                assertThat(state.getVelocity(), closeTo(lastState.getVelocity() + (state.t() - lastState.t()) * lastState.getAcceleration(), Epsilon));
            }
        }
    }

    @Test
    public void testNoConstraints() {
        Trajectory<Translation> trajectory = new Trajectory<>(WAYPOINTS);
        DistanceView<Translation> distanceView = new DistanceView<>(trajectory);

        final var cfg1 = new MotionPlanner.MotionConfig() {
            {
                maxVelocity = 20.0;
                maxAcceleration = 5.0;
            }
        };
        // Triangle profile.
        Trajectory<TimedState<Translation>> timedTrajectory = buildAndCheckTrajectory(distanceView, 1.0, new ArrayList<TimingConstraint<Translation>>(), cfg1,0.0, 0.0);

        final var cfg2 = new MotionPlanner.MotionConfig() {
            {
                maxVelocity = 10.0;
                maxAcceleration = 0.5;
            }
        };
        // Trapezoidal profile.
        timedTrajectory = buildAndCheckTrajectory(distanceView, 1.0, new ArrayList<TimingConstraint<Translation>>(), cfg2, 0.0, 0.0);

        final var cfg3 = new MotionPlanner.MotionConfig() {
            {
                maxVelocity = 10.0;
                maxAcceleration = 5.0;
            }
        };
        // Trapezoidal profile with start and end velocities.
        timedTrajectory = buildAndCheckTrajectory(distanceView, 1.0, new ArrayList<TimingConstraint<Translation>>(), cfg3, 5.0, 2.0);
    }

    @Test
    public void testConditionalVelocityConstraint() {
        Trajectory<Translation> translation = new Trajectory<>(WAYPOINTS);
        DistanceView<Translation> distanceView = new DistanceView<>(translation);

        class ConditionalTimingConstraint<S extends ITranslation2d<S>> implements TimingConstraint<S> {
            @Override
            public double getMaxVelocity(final S state) {
                if (state.getTranslation().x() >= 24.0) {
                    return 5.0;
                } else {
                    return Double.POSITIVE_INFINITY;
                }
            }

            @Override
            public TimingConstraint.MinMaxAcceleration getMinMaxAcceleration(final S state, final double velocity) {
                return new TimingConstraint.MinMaxAcceleration(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
            }
        }

        final var cfg = new MotionPlanner.MotionConfig() {
            {
                maxVelocity = 10.0;
                maxAcceleration = 5.0;
            }
        };

        // Trapezoidal profile.
        Trajectory<TimedState<Translation>> timedTrajectory = buildAndCheckTrajectory(
                distanceView,
                1.0,
                Collections.singletonList(new ConditionalTimingConstraint<>()),
                cfg,
                0.0,
                0.0);
    }

    @Test
    public void testConditionalAccelerationConstraint() {
        Trajectory<Translation> trajectory = new Trajectory<>(WAYPOINTS);
        DistanceView<Translation> distanceView = new DistanceView<>(trajectory);

        class ConditionalTimingConstraint<S extends ITranslation2d<S>> implements TimingConstraint<S> {
            @Override
            public double getMaxVelocity(S state) {
                return Double.POSITIVE_INFINITY;
            }

            @Override
            public TimingConstraint.MinMaxAcceleration getMinMaxAcceleration(S state,
                                                                             double velocity) {
                return new TimingConstraint.MinMaxAcceleration(-10.0, 10.0 / velocity);
            }
        }

        final var cfg = new MotionPlanner.MotionConfig() {
            {
                maxVelocity = 10.0;
                maxAcceleration = 5.0;
            }
        };

        // Trapezoidal profile.
        Trajectory<TimedState<Translation>> timedTrajectory = buildAndCheckTrajectory(
                distanceView,
                1.0,
                Collections.singletonList(new ConditionalTimingConstraint<>()),
                cfg,
                0.0,
                0.0);
    }

    @Test
    public void testVelocityLimitRegionConstraint() {
        Trajectory<Translation> trajectory = new Trajectory<>(WAYPOINTS);
        DistanceView<Translation> distanceView = new DistanceView<>(trajectory);

        VelocityLimitRegionConstraint<Translation> constraint = new VelocityLimitRegionConstraint<>(new Translation(6.0, -6.0), new Translation(18.0, 6.0), 3.0);

        final var cfg = new MotionPlanner.MotionConfig() {
            {
                maxVelocity = 10.0;
                maxAcceleration = 5.0;
            }
        };

        // Trapezoidal profile.
        Trajectory<TimedState<Translation>> timedTrajectory = buildAndCheckTrajectory(
                distanceView,
                1.0,
                Collections.singletonList(constraint),
                cfg,
                0.0,
                0.0);
    }
}
