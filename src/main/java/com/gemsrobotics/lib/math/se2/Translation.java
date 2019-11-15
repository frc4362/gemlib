package com.gemsrobotics.lib.math.se2;

import com.gemsrobotics.lib.utils.FastDoubleToString;
import com.gemsrobotics.lib.utils.MathUtils;
import com.google.gson.annotations.SerializedName;

import java.util.Objects;

import static java.lang.Math.*;

/**
 * A translation in a 2d coordinate frame. Translations are simply shifts in an (x, y) plane.
 */
@SuppressWarnings({"unused", "WeakerAccess"})
public class Translation implements ITranslation2d<Translation> {
    public static Translation identity() {
        return new Translation();
    }

    @SerializedName("x")
    protected final double m_x;
    @SerializedName("y")
    protected final double m_y;

    public Translation(double x, double y) {
        m_x = x;
        m_y = y;
    }

    public Translation(final Translation other) {
        this(other.m_x, other.m_y);
    }

    public Translation() {
        this(0, 0);
    }

    public Translation(final Translation start, final Translation end) {
        this(end.m_x - start.m_x, end.m_y - start.m_y);
    }

    public static Translation fromPolar(final Rotation direction, final double magnitude) {
        return new Translation(direction.cos() * magnitude, direction.sin() * magnitude);
    }

    /**
     * The "norm" of a transform is the Euclidean distance in x and y.
     * Otherwise known as the magnitude or the length.
     *
     * @return sqrt(x ^ 2 + y ^ 2)
     */
    public final double norm() {
        return hypot(m_x, m_y);
    }

    public final double x() {
        return m_x;
    }

    public final double y() {
        return m_y;
    }

    /**
     * We can compose Translation2d's by adding together the x and y shifts.
     *
     * @param other The other translation to add.
     * @return The combined effect of translating by this object and the other.
     */
    public Translation translateBy(final Translation other) {
        return new Translation(m_x + other.m_x, m_y + other.m_y);
    }

    /**
     * We can also rotate Translation2d's. See: https://en.wikipedia.org/wiki/Rotation_matrix
     *
     * @param rotation The rotation to apply.
     * @return This translation rotated by rotation.
     */
    public Translation rotateBy(final Rotation rotation) {
        return new Translation((m_x * rotation.cos()) - (m_y * rotation.sin()), (m_x * rotation.sin()) + (m_y * rotation.cos()));
    }

    public Translation sum(final Translation other) {
        return translateBy(other);
    }

    public Translation difference(final Translation other) {
        return translateBy(other.inverse());
    }

    public Rotation direction() {
        return new Rotation(m_x, m_y, true);
    }

    /**
     * The inverse simply means a Translation2d that "undoes" this object.
     *
     * @return Translation by -x and -y.
     */
    public Translation inverse() {
        return new Translation(-m_x, -m_y);
    }

    public Translation normalized() {
        if (epsilonEquals(identity(), MathUtils.Epsilon)) {
            return this;
        }

        return scale(1.0 / norm());
    }

    @Override
    public Translation interpolate(final Translation other, final double n) {
        if (n <= 0) {
            return new Translation(this);
        } else if (n >= 1) {
            return new Translation(other);
        }

        return extrapolate(other, n);
    }

    public Translation extrapolate(final Translation other, final double n) {
        return new Translation(n * (other.m_x - m_x) + m_x, n * (other.m_y - m_y) + m_y);
    }

    public Translation scale(final double s) {
        return new Translation(m_x * s, m_y * s);
    }

    public static double dot(final Translation a, final Translation b) {
        return (a.m_x * b.m_x) + (a.m_y * b.m_y);
    }

    public static Rotation getAngle(final Translation a, final Translation b) {
        final double cosAngle = dot(a, b) / (a.norm() * b.norm());

        if (Double.isNaN(cosAngle)) {
            return new Rotation();
        }

        return Rotation.radians(acos(min(1.0, max(cosAngle, -1.0))));
    }

    public static double cross(final Translation a, final Translation b) {
        return (a.m_x * b.m_y) - (a.m_y * b.m_x);
    }

    /**
     * The scalar projection of a vector u onto a vector v is the length of
     * the "shadow" cast by u onto v under a "light" that is placed on a line
     * normal to v and containing the endpoint of u, given that u and v share
     * a starting point.
     * tl;dr:
     *    _*
     * u  /|
     *   / |
     *  /  |       v
     * *---+---------------->*
     * \___/
     *   |
     *  scal_v(u)
     * u.scal(v)
     *
     * @return (u . v) / norm(v)
     */
    public double scal(final Translation other) {
        return dot(this, other) / other.norm();
    }

    /**
     * The projection of a vector u onto a vector v is the vector in the direction
     * of v with the magnitude u.scal(v).
     *
     * @return u.scal(v) * v / norm(v)
     */
    public Translation project(final Translation other) {
        return other.normalized().scale(scal(other));
    }

    /**
     * https://stackoverflow.com/a/1167047/6627273
     * A point D is considered "within" an angle ABC when cos(DBM) > cos(ABM)
     * where M is the midpoint of AC, so ABM is half the angle ABC.
     * The cosine of an angle can be computed as the dot product of two normalized
     * vectors in the directions of its sides.
     * Note that this definition of "within" does not include points that lie on
     * the sides of the given angle.
     * If `vertical` is true, then check not within the given angle, but within the
     * image of that angle rotated by pi about its vertex.
     *
     * @param a A point on one side of the angle.
     * @param vertex The vertex of the angle.
     * @param c A point on the other side of the angle.
     * @param vertical Whether to check in the angle vertical to the one given
     *
     * @return Whether this translation is within the given angle.
     * @author Joseph Reed 1323
     */
    public boolean isWithinAngle(final Translation a, final Translation vertex, final Translation c, final boolean vertical) {
        final var midpoint = a.interpolate(c, 0.5);
        var center = (new Translation(vertex, midpoint)).normalized();
        var side = (new Translation(vertex, a)).normalized();
        final var distance = (new Translation(vertex, this)).normalized();

        if (vertical) {
            center = center.inverse();
            side = side.inverse();
        }

        return dot(distance, midpoint) > dot(side, center);
    }

    public boolean isWithinAngle(final Translation a, final Translation b, final Translation c) {
        return isWithinAngle(a, b, c,false);
    }

    /** Assumes an angle centered at the origin. */
    public boolean isWithinAngle(final Translation a, final Translation c, final boolean vertical) {
        return isWithinAngle(a, identity(), a, vertical);
    }

    public boolean isWithinAngle(final Translation a, final Translation c) {
        return isWithinAngle(a, c, false);
    }

    public boolean epsilonEquals(final Translation other, final double epsilon) {
        return MathUtils.epsilonEquals(x(), other.x(), epsilon) && MathUtils.epsilonEquals(y(), other.y(), epsilon);
    }

    @Override
    public double distance(final Translation other) {
        return inverse().translateBy(other).norm();
    }

    @Override
    public Translation getTranslation() {
        return this;
    }

    @Override
    public boolean equals(final Object other) {
        if (!(other instanceof Translation)) {
            return false;
        }

        return MathUtils.epsilonEquals(distance((Translation) other), 0);
    }

    @Override
    public String toString() {
        return "(" + FastDoubleToString.format(m_x, 3) + ", " + FastDoubleToString.format(m_y, 3) + ")";
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_x, m_y);
    }
}
