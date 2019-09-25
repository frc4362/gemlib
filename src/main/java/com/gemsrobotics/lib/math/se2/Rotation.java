package com.gemsrobotics.lib.math.se2;

import com.gemsrobotics.lib.utils.FastDoubleToString;
import com.gemsrobotics.lib.utils.MathUtils;
import com.google.gson.annotations.SerializedName;

import java.util.Objects;

import static com.gemsrobotics.lib.utils.MathUtils.epsilonEquals;
import static java.lang.Math.hypot;

/**
 * A rotation in a 2d coordinate frame represented a point on the unit circle (cosine and sine).
 * <p>
 * Inspired by Sophus (https://github.com/strasdat/Sophus/tree/master/sophus)
 */
@SuppressWarnings({"unused", "WeakerAccess"})
public class Rotation implements IRotation2d<Rotation> {
    public static Rotation identity() {
        return new Rotation();
    }

    @SerializedName("cos")
    protected final double m_cos;
    @SerializedName("sin")
    protected final double m_sin;

    public Rotation() {
        this(1, 0, false);
    }

    public Rotation(final double x, final double y, final boolean normalize) {
        if (normalize) {
            // From trig, we know that sin^2 + cos^2 == 1, but as we do math on this object we might accumulate rounding errors.
            // Normalizing forces us to re-scale the sin and cos to reset rounding errors.
            final double magnitude = hypot(x, y);

            if (magnitude > MathUtils.kEpsilon) {
                m_sin = y / magnitude;
                m_cos = x / magnitude;
            } else {
                m_sin = 0;
                m_cos = 1;
            }
        } else {
            m_cos = x;
            m_sin = y;
        }
    }

    public Rotation(final Rotation other) {
        this(other.m_cos, other.m_sin, false);
    }

    public Rotation(final Translation direction, boolean normalize) {
        this(direction.x(), direction.y(), normalize);
    }

    public static Rotation radians(final double angleRadians) {
        return new Rotation(Math.cos(angleRadians), Math.sin(angleRadians), false);
    }

    public static Rotation degrees(final double angleDegrees) {
        return radians(Math.toRadians(angleDegrees));
    }

    public double cos() {
        return m_cos;
    }

    public double sin() {
        return m_sin;
    }

    public double tan() {
        if (epsilonEquals(m_cos, 0)) {
            if (m_sin >= 0.0) {
                return Double.POSITIVE_INFINITY;
            } else {
                return Double.NEGATIVE_INFINITY;
            }
        }

        return m_sin / m_cos;
    }

    public double getRadians() {
        return Math.atan2(m_sin, m_cos);
    }

    public double getDegrees() {
        return Math.toDegrees(getRadians());
    }

    /**
     * We can rotate this Rotation2d by adding together the effects of it and another rotation.
     *
     * @param other The other rotation. See: https://en.wikipedia.org/wiki/Rotation_matrix
     * @return This rotation rotated by other.
     */
    public Rotation rotateBy(final Rotation other) {
        return new Rotation(
                m_cos * other.m_cos - m_sin * other.m_sin,
                m_cos * other.m_sin + m_sin * other.m_cos,
                true
        );
    }

    public Rotation sum(final Rotation other) {
        return rotateBy(other);
    }

    public Rotation difference(final Rotation other) {
        return rotateBy(other.inverse());
    }

    public Rotation normal() {
        return new Rotation(-m_sin, m_cos, false);
    }

    /**
     * The inverse of a Rotation2d "undoes" the effect of this rotation.
     *
     * @return The opposite of this rotation.
     */
    public Rotation inverse() {
        return new Rotation(m_cos, -m_sin, false);
    }

    public boolean isParallel(final Rotation other) {
        return epsilonEquals(Translation.cross(toTranslation(), other.toTranslation()), 0.0);
    }

    public Translation toTranslation() {
        return new Translation(m_cos, m_sin);
    }

    @Override
    public Rotation interpolate(final Rotation other, final double x) {
        if (x <= 0) {
            return new Rotation(this);
        } else if (x >= 1) {
            return new Rotation(other);
        }

        final double angleDiff = inverse().rotateBy(other).getRadians();

        return rotateBy(Rotation.radians(angleDiff * x));
    }

    @Override
    public String toString() {
        return "(" + FastDoubleToString.format(getDegrees(), 3) + " deg)";
    }

    @Override
    public double distance(final Rotation other) {
        return inverse().rotateBy(other).getRadians();
    }

    @Override
    public boolean equals(final Object other) {
        if (Objects.isNull(other) || !(other instanceof IRotation2d)) {
            return false;
        }

        return epsilonEquals(distance((Rotation) other), 0);
    }

    @Override
    public Rotation getRotation() {
        return this;
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_sin, m_cos);
    }
}
