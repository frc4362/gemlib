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

    /**
     * The "norm" of a transform is the Euclidean distance in x and y.
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
        return new Translation(m_x * rotation.cos() - m_y * rotation.sin(), m_x * rotation.sin() + m_y * rotation.cos());
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

    @Override
    public Translation interpolate(final Translation other, final double x) {
        if (x <= 0) {
            return new Translation(this);
        } else if (x >= 1) {
            return new Translation(other);
        }

        return extrapolate(other, x);
    }

    public Translation extrapolate(final Translation other, final double x) {
        return new Translation(x * (other.m_x - m_x) + m_x, x * (other.m_y - m_y) + m_y);
    }

    public Translation scale(final double s) {
        return new Translation(m_x * s, m_y * s);
    }

    public boolean epsilonEquals(final Translation other, final double epsilon) {
        return MathUtils.epsilonEquals(x(), other.x(), epsilon) && MathUtils.epsilonEquals(y(), other.y(), epsilon);
    }

    @Override
    public String toString() {
        return "(" + FastDoubleToString.format(m_x, 3) + ", " + FastDoubleToString.format(m_y, 3) + ")";
    }

    public static double dot(final Translation a, final Translation b) {
        return a.m_x * b.m_x + a.m_y * b.m_y;
    }

    public static Rotation getAngle(final Translation a, final Translation b) {
        final double cosAngle = dot(a, b) / (a.norm() * b.norm());

        if (Double.isNaN(cosAngle)) {
            return new Rotation();
        }

        return Rotation.radians(acos(min(1.0, max(cosAngle, -1.0))));
    }

    public static double cross(final Translation a, final Translation b) {
        return a.m_x * b.m_y - a.m_y * b.m_x;
    }

    @Override
    public double distance(final Translation other) {
        return inverse().translateBy(other).norm();
    }

    @Override
    public boolean equals(final Object other) {
        if (Objects.isNull(other) || !(other instanceof Translation)) {
            return false;
        }

        return MathUtils.epsilonEquals(distance((Translation) other), 0);
    }

    @Override
    public Translation getTranslation() {
        return this;
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_x, m_y);
    }
}
