package com.gemsrobotics.lib.math.spline;

import com.gemsrobotics.lib.controls.DriveMotionPlanner;
import com.gemsrobotics.lib.math.se2.*;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.abs;

public class SplineGenerator {
    private static final int kMinSampleSize = 1;

    /**
     * Converts a spline into a list of Twist2d's.
     *
     * @param s  the spline to parametrize
     * @param t0 starting percentage of spline to parametrize
     * @param t1 ending percentage of spline to parametrize
     * @return list of RigidTransform2dWithCurvature
     * that approximates the original spline
     */
    private static List<RigidTransformWithCurvature> parameterizeSpline(
            final Spline s,
            final DriveMotionPlanner.MotionConfig config,
            final double t0,
            final double t1
    ) {
        final var rv = new ArrayList<RigidTransformWithCurvature>();

        rv.add(s.getRigidTransformWithCurvature(0.0));

        final var dt = (t1 - t0);

        for (double t = 0; t < t1; t += dt / kMinSampleSize) {
            makeSegmentArc(s, rv, t, t + dt / kMinSampleSize, config.maxDx, config.maxDy, config.maxDtheta);
        }

        return rv;
    }

    public static List<RigidTransformWithCurvature> parameterizeSpline(final DriveMotionPlanner.MotionConfig config, final Spline s) {
        return parameterizeSpline(s, config, 0.0, 1.0);
    }

    public static List<RigidTransformWithCurvature> parameterizeSplines(
            final DriveMotionPlanner.MotionConfig config,
            final List<? extends Spline> splines
    ) {
        final var rv = new ArrayList<RigidTransformWithCurvature>();

        if (splines.isEmpty()) {
            return rv;
        }

        rv.add(splines.get(0).getRigidTransformWithCurvature(0.0));

        for (final var spline : splines) {
            final var samples = parameterizeSpline(config, spline);
            samples.remove(0);

            rv.addAll(samples);
        }

        return rv;
    }

    private static void makeSegmentArc(
            final Spline spline,
            final List<RigidTransformWithCurvature> points,
            final double t0,
            final double t1,
            final double maxDx,
            final double maxDy,
            final double maxDTheta
    ) {
        final var p0 = spline.getPoint(t0);
        final var p1 = spline.getPoint(t1);
        final var r0 = spline.getHeading(t0);
        final var r1 = spline.getHeading(t1);
        final var transformation = new RigidTransform(new Translation(p0, p1).rotateBy(r0.inverse()), r1.difference(r0));

        final var twist = transformation.toTwist();
        // thank you to the man Prateek of 5190 for finding this bug
        if (abs(twist.dy) > maxDy || abs(twist.dx) > maxDx || abs(twist.dtheta) > maxDTheta) {
            makeSegmentArc(spline, points, t0, (t0 + t1) / 2, maxDx, maxDy, maxDTheta);
            makeSegmentArc(spline, points, (t0 + t1) / 2, t1, maxDx, maxDy, maxDTheta);
        } else {
            points.add(spline.getRigidTransformWithCurvature(t1));
        }
    }
}
