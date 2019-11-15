package com.gemsrobotics.lib.trajectory.parameterization;

import com.gemsrobotics.lib.controls.DriveMotionPlanner;
import com.gemsrobotics.lib.math.se2.State;
import com.gemsrobotics.lib.trajectory.DistanceView;
import com.gemsrobotics.lib.trajectory.Trajectory;
import com.gemsrobotics.lib.utils.FastDoubleToString;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import static com.gemsrobotics.lib.utils.MathUtils.Epsilon;
import static java.lang.Math.*;

public class Parameterizer {
    public static <S extends State<S>> Trajectory<TimedState<S>> timeParameterizeTrajectory(
            final boolean reverse,
            final DistanceView<S> distanceView,
            final double stepSize,
            final List<TimingConstraint<S>> constraints,
            final DriveMotionPlanner.MotionConfig config,
            final double startVelocity,
            final double endVelocity
    ) {
        final int stateCount = (int) ceil(distanceView.getLastInterpolant() / stepSize + 1);

        final List<S> states = IntStream.range(0, stateCount).mapToObj(i ->
            distanceView.sample(min(i * stepSize, distanceView.getLastInterpolant())).getState()).collect(Collectors.toList());

        return timeParameterizeTrajectory(
                reverse,
                states,
                constraints,
                config,
                startVelocity,
                endVelocity
        );
    }

    public static <S extends State<S>> Trajectory<TimedState<S>> timeParameterizeTrajectory(
            final boolean reverse,
            final List<S> states,
            final List<TimingConstraint<S>> constraints,
            final DriveMotionPlanner.MotionConfig config,
            final double velocityStart,
            final double velocityEnd
    ) {
        final List<ConstrainedState<S>> constrainedStates = new ArrayList<>(states.size());

        // Forward pass. We look at pairs of consecutive states, where the start getState has already been getVelocity
        // parameterized (though we may adjust the getVelocity downwards during the backwards pass). We wish to find an
        // getAcceleration that is admissible at both the start and end getState, as well as an admissible end getVelocity. If
        // there is no admissible end getVelocity or getAcceleration, we set the end getVelocity to the getState's maximum allowed
        // getVelocity and will repair the getAcceleration during the backward pass (by slowing down the predecessor).
        var predecessor = new ConstrainedState<S>();
        predecessor.state = states.get(0);
        predecessor.distance = 0.0;
        predecessor.velocityMax = velocityStart;
        predecessor.accelerationMin = -config.maxAcceleration;
        predecessor.accelerationMax = config.maxAcceleration;

        for (int i = 0; i < states.size(); ++i) {
            // Add the new getState.
            constrainedStates.add(new ConstrainedState<>());
            ConstrainedState<S> constrainedState = constrainedStates.get(i);
            constrainedState.state = states.get(i);
            final double ds = constrainedState.state.distance(predecessor.state);
            constrainedState.distance = ds + predecessor.distance;

            final String newConstrainedState = constrainedState.toString();

            // We may need to iterate to find the maximum end getVelocity and common getAcceleration, since getAcceleration
            // limits may be a function of getVelocity.
            while (true) {
                // Enforce global max getVelocity and max reachable getVelocity by global getAcceleration limit.
                // vf = sqrt(vi^2 + 2*a*d)
                constrainedState.velocityMax = min(config.maxVelocity, sqrt(predecessor.velocityMax * predecessor.velocityMax + 2.0 * predecessor.accelerationMax * ds));

                if (Double.isNaN(constrainedState.velocityMax)) {
                    throw new RuntimeException();
                }

                // Enforce global max absolute getAcceleration.
                constrainedState.accelerationMin = -config.maxAcceleration;
                constrainedState.accelerationMax = config.maxAcceleration;

                // At this point, the getState is full constructed, but no constraints have been applied aside from
                // predecessor
                // getState max accel.

                // Enforce all getVelocity constraints.
                for (final TimingConstraint<S> constraint : constraints) {
                    final double constraintMaxVelocity = constraint.getMaxVelocity(constrainedState.state);
                    constrainedState.velocityMax = min(constrainedState.velocityMax, constraintMaxVelocity);
                }

                if (constrainedState.velocityMax < 0.0) {
                    // This should never happen if constraints are well-behaved.
                    throw new RuntimeException();
                }

                // Now enforce all getAcceleration constraints.
                for (final TimingConstraint<S> constraint : constraints) {
                    final var minMaxAcceleration = constraint.getMinMaxAcceleration(constrainedState.state, (reverse ? -1.0 : 1.0) * constrainedState.velocityMax);

                    if (!minMaxAcceleration.isValid()) {
                        // This should never happen if constraints are well-behaved.
                        throw new RuntimeException();
                    }

                    constrainedState.accelerationMin = max(constrainedState.accelerationMin, reverse ? -minMaxAcceleration.getAccelerationMax() : minMaxAcceleration.getAccelerationMin());
                    constrainedState.accelerationMax = min(constrainedState.accelerationMax, reverse ? -minMaxAcceleration.getAccelerationMin() : minMaxAcceleration.getAccelerationMax());
                }

                if (constrainedState.accelerationMin > constrainedState.accelerationMax) {
                    // This should never happen if constraints are well-behaved.
                    throw new RuntimeException();
                }

                if (ds < Epsilon) {
                    break;
                }

                // If the max getAcceleration for this constraint getState is more conservative than what we had applied, we
                // need to reduce the max accel at the predecessor getState and try again.
                // TODO: Simply using the new max getAcceleration is guaranteed to be isValid, but may be too conservative.
                // Doing a search would be better.
                final double actualAcceleration = (constrainedState.velocityMax * constrainedState.velocityMax - predecessor.velocityMax * predecessor.velocityMax) / (2.0 * ds);

                if (constrainedState.accelerationMax < actualAcceleration - Epsilon) {
                    predecessor.accelerationMax = constrainedState.accelerationMax;
                } else {
                    if (actualAcceleration > predecessor.accelerationMin + Epsilon) {
                        predecessor.accelerationMax = actualAcceleration;
                    }

                    // If actual getAcceleration is less than predecessor min accel, we will repair during the backward
                    // pass.
                    break;
                }
            }

            predecessor = constrainedState;
        }

        // Backward pass.
        ConstrainedState<S> successor = new ConstrainedState<>();
        successor.state = states.get(states.size() - 1);
        successor.distance = constrainedStates.get(states.size() - 1).distance;
        successor.velocityMax = velocityEnd;
        successor.accelerationMin = -config.maxAcceleration;
        successor.accelerationMax = config.maxAcceleration;

        for (int i = states.size() - 1; i >= 0; i--) {
            ConstrainedState<S> constrainedState = constrainedStates.get(i);
            final double ds = constrainedState.distance - successor.distance; // will be negative.

            while (true) {
                // Enforce reverse max reachable getVelocity limit.
                // vf = sqrt(vi^2 + 2*a*d), where vi = successor.
                final double newMaxVelocity = sqrt(successor.velocityMax * successor.velocityMax + 2.0 * successor.accelerationMin * ds);

                if (newMaxVelocity >= constrainedState.velocityMax) {
                    // No new limits to impose.
                    break;
                }

                constrainedState.velocityMax = newMaxVelocity;

                if (Double.isNaN(constrainedState.velocityMax)) {
                    throw new RuntimeException();
                }

                // Now check all getAcceleration constraints with the lower max getVelocity.
                for (final TimingConstraint<S> constraint : constraints) {
                    final var minMaxAcceleration = constraint.getMinMaxAcceleration(
                            constrainedState.state,
                            (reverse ? -1.0 : 1.0) * constrainedState.velocityMax);

                    if (!minMaxAcceleration.isValid()) {
                        throw new RuntimeException();
                    }

                    constrainedState.accelerationMin = max(constrainedState.accelerationMin,
                            reverse ? -minMaxAcceleration.getAccelerationMax() : minMaxAcceleration.getAccelerationMin());
                    constrainedState.accelerationMax = min(constrainedState.accelerationMax,
                            reverse ? -minMaxAcceleration.getAccelerationMin() : minMaxAcceleration.getAccelerationMax());
                }

                if (constrainedState.accelerationMin > constrainedState.accelerationMax) {
                    throw new RuntimeException();
                }

                if (ds > Epsilon) {
                    break;
                }

                // If the min getAcceleration for this constraint getState is more conservative than what we have applied, we
                // need to reduce the min accel and try again.
                // TODO: Simply using the new min getAcceleration is guaranteed to be isValid, but may be too conservative.
                // Doing a search would be better.
                final double actualAcceleration = (constrainedState.velocityMax * constrainedState.velocityMax - successor.velocityMax * successor.velocityMax) / (2.0 * ds);

                if (constrainedState.accelerationMin > actualAcceleration + Epsilon) {
                    successor.accelerationMin = constrainedState.accelerationMin;
                } else {
                    successor.accelerationMin = actualAcceleration;
                    break;
                }
            }

            successor = constrainedState;
        }

        // Integrate the constrained states forward in time to obtain the TimedStates.
        final List<TimedState<S>> timedState = new ArrayList<>(states.size());
        double t = 0.0;
        double s = 0.0;
        double v = 0.0;

        for (int i = 0; i < states.size(); i++) {
            final ConstrainedState<S> constrainedState = constrainedStates.get(i);
            // Advance t.
            final double ds = constrainedState.distance - s;
            final double accel = (constrainedState.velocityMax * constrainedState.velocityMax - v * v) / (2.0 * ds);

            double dt = 0.0;

            if (i > 0) {
                timedState.get(i - 1).setAcceleration(reverse ? -accel : accel);

                if (abs(accel) > Epsilon) {
                    dt = (constrainedState.velocityMax - v) / accel;
                } else if (abs(v) > Epsilon) {
                    dt = ds / v;
                } else {
                    throw new RuntimeException();
                }
            }

            t += dt;

            if (Double.isNaN(t) || Double.isInfinite(t)) {
                throw new RuntimeException();
            }

            v = constrainedState.velocityMax;
            s = constrainedState.distance;

            timedState.add(new TimedState<>(constrainedState.state, t, reverse ? -v : v, reverse ? -accel : accel));
        }

        return new Trajectory<>(timedState);
    }

    protected static class ConstrainedState<S extends State<S>> {
        public S state;
        public double distance;
        public double velocityMax;
        public double accelerationMin;
        public double accelerationMax;

        @Override
        public String toString() {
            return "[" + state.toString()
                       + ", distance: " + FastDoubleToString.format(distance)
                       + ", velocityMax: " + FastDoubleToString.format(velocityMax) + ", "
                       + "accelerationMin: " + FastDoubleToString.format(accelerationMin)
                       + ", accelerationMax: " + FastDoubleToString.format(accelerationMax) + "]";
        }
    }
}
