package com.gemsrobotics.lib.trajectory.parameterization;

import com.gemsrobotics.lib.controls.DriveMotionPlanner;
import com.gemsrobotics.lib.math.se2.*;
import com.gemsrobotics.lib.math.spline.QuinticHermiteSpline;
import com.gemsrobotics.lib.math.spline.Spline;
import com.gemsrobotics.lib.math.spline.SplineGenerator;
import com.gemsrobotics.lib.trajectory.Trajectory;
import com.gemsrobotics.lib.trajectory.TrajectoryView;
import com.gemsrobotics.lib.utils.MathUtils;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.ceil;

public class TrajectoryUtils {
    public static <S extends IRigidTransform2d<S>> Trajectory<S> mirror(final Trajectory<S> trajectory) {
        final var waypoints = new ArrayList<S>(trajectory.length());

        for (int i = 0; i < trajectory.length(); i++) {
            waypoints.add(trajectory.getState(i).mirror());
        }

        return new Trajectory<>(waypoints);
    }

    public static <S extends IRigidTransform2d<S>> Trajectory<TimedState<S>> mirrorTimed(final Trajectory<TimedState<S>> trajectory) {
        final var waypoints = new ArrayList<TimedState<S>>(trajectory.length());

        for (int i = 0; i < trajectory.length(); i++) {
            final var timedState = trajectory.getState(i);
            waypoints.add(new TimedState<>(
                    timedState.getState().mirror(),
                    timedState.t(),
                    timedState.getVelocity(),
                    timedState.getAcceleration()));
        }

        return new Trajectory<>(waypoints);
    }

    public static <S extends IRigidTransform2d<S>> Trajectory<S> transform(final Trajectory<S> trajectory, final RigidTransform transform) {
        List<S> waypoints = new ArrayList<>(trajectory.length());
        for (int i = 0; i < trajectory.length(); ++i) {
            waypoints.add(trajectory.getState(i).transformBy(transform));
        }
        return new Trajectory<>(waypoints);
    }

    /**
     * Creates a Trajectory by sampling a TrajectoryView at a regular interval.
     */
    public static <S extends State<S>> Trajectory<S> resample(final TrajectoryView<S> trajectoryView, final double interval) {
        if (interval <= MathUtils.kEpsilon) {
            return new Trajectory<>();
        }

        final int num_states = (int) ceil((trajectoryView.getLastInterpolant() - trajectoryView.getFirstInterpolant()) / interval);
        final var states = new ArrayList<S>(num_states);

        for (int i = 0; i < num_states; i++) {
            states.add(trajectoryView.sample(i * interval + trajectoryView.getFirstInterpolant()).state());
        }

        return new Trajectory<>(states);
    }

    public static Trajectory<RigidTransformWithCurvature> trajectoryFromSplineWaypoints(
    	final List<RigidTransform> waypoints,
        final DriveMotionPlanner.MotionConfig config
	) {
        List<QuinticHermiteSpline> splines = new ArrayList<>(waypoints.size() - 1);
        for (int i = 1; i < waypoints.size(); ++i) {
            splines.add(new QuinticHermiteSpline(waypoints.get(i - 1), waypoints.get(i)));
        }

        QuinticHermiteSpline.optimizeSpline(splines);

        return trajectoryFromSplines(splines, config);
    }

    public static Trajectory<RigidTransformWithCurvature> trajectoryFromSplines(
            final List<? extends Spline> splines,
            final DriveMotionPlanner.MotionConfig config
    ) {
        return new Trajectory<>(SplineGenerator.parameterizeSplines(config, splines));
    }
}
