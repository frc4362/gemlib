package com.gemsrobotics.lib.math.se2;

import com.gemsrobotics.lib.utils.FastDoubleToString;

import java.util.Objects;

import static com.gemsrobotics.lib.utils.MathUtils.epsilonEquals;
import static java.lang.Math.*;

/**
 * A movement along an arc at constant curvature and velocity. We can use ideas from differential calculus to create
 * new RigidTransform2d's from a Twist2d and visa versa.
 * <p>
 * A Twist can be used to represent a difference between two poses, a getVelocity, an getAcceleration, etc.
 */
@SuppressWarnings({"unused", "WeakerAccess"})
public class Twist {
    public static Twist identity() {
        return new Twist(0.0, 0.0, 0.0);
    }

    public final double dx;
    public final double dy;
    public final double dtheta; // Radians!

    public Twist(final double dx, final double dy, final double dtheta) {
        this.dx = dx;
        this.dy = dy;
        this.dtheta = dtheta;
    }

    public Twist scaled(final double scale) {
        return new Twist(dx * scale, dy * scale, dtheta * scale);
    }

    public double norm() {
        // Common case of dy == 0
        if (dy == 0.0) {
            return abs(dx);
        }

        return hypot(dx, dy);
    }

    public double curvature() {
        if (epsilonEquals(dtheta, 0) && epsilonEquals(norm(), 0)) {
            return 0.0;
        }

        return dtheta / norm();
    }

    @Override
    public String toString() {
        return "(" + FastDoubleToString.format(dx, 3) + ","
                   + FastDoubleToString.format(dy, 3) + ","
                   + FastDoubleToString.format(toDegrees(dtheta), 3) + " deg)";
    }

    public final RigidTransform toRigidTransform() {
        return RigidTransform.ofTwist(this);
    }

    @Override
    public int hashCode() {
        return Objects.hash(dx, dy, dtheta);
    }
}