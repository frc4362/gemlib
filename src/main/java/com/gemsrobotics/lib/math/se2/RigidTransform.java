package com.gemsrobotics.lib.math.se2;

import com.gemsrobotics.lib.utils.MathUtils;
import com.google.gson.annotations.SerializedName;

import java.util.Objects;

import static java.lang.Math.abs;
import static java.lang.Math.sin;
import static java.lang.StrictMath.cos;

/**
 * Represents a 2d rigid transform containing translational and rotational elements.
 * <p>
 * Inspired by Sophus (https://github.com/strasdat/Sophus/tree/master/sophus)
 */
@SuppressWarnings({"unused", "WeakerAccess"})
public class RigidTransform implements IRigidTransform2d<RigidTransform> {
    public static RigidTransform identity() {
        return new RigidTransform();
    }

    @SerializedName("translation")
    protected final Translation m_translation;
    @SerializedName("rotation")
    protected final Rotation m_rotation;

    public RigidTransform(final Translation translation, final Rotation rotation) {
        m_translation = translation;
        m_rotation = rotation;
    }

    public RigidTransform() {
        this(Translation.identity(), Rotation.identity());
    }

    public RigidTransform(final double x, final double y, final Rotation rotation) {
        this(new Translation(x, y), rotation);
    }

    public RigidTransform(final RigidTransform other) {
        this(new Translation(other.m_translation), new Rotation(other.m_rotation));
    }

    public static RigidTransform fromTranslation(final Translation translation) {
        return new RigidTransform(translation, new Rotation());
    }

    public static RigidTransform fromRotation(final Rotation rotation) {
        return new RigidTransform(new Translation(), rotation);
    }

    /**
     * Obtain a new RigidTransform2d from a (constant curvature) getVelocity. See:
     * https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp
     */
    public static RigidTransform ofTwist(final Twist delta) {
        final double sinTheta = sin(delta.dtheta);
        final double cosTheta = cos(delta.dtheta);

        final double s, c;

        if (MathUtils.epsilonEquals(delta.dtheta, 0)) {
            s = 1.0 - 1.0 / 6.0 * delta.dtheta * delta.dtheta;
            c = .5 * delta.dtheta;
        } else {
            s = sinTheta / delta.dtheta;
            c = (1.0 - cosTheta) / delta.dtheta;
        }

        return new RigidTransform(
                new Translation(
                        delta.dx * s - delta.dy * c,
                        delta.dx * c + delta.dy * s),
                new Rotation(
                        cosTheta,
                        sinTheta,
                        false));
    }

    /**
     * Logical inverse of the above.
     */
    public static Twist toTwist(final RigidTransform transform) {
        final double dtheta = transform.getRotation().getRadians();
        final double halfDtheta = 0.5 * dtheta;
        final double cosMinusOne = transform.getRotation().cos() - 1.0;

        final double halfthetaByTanOfHalfdtheta;

        if (MathUtils.epsilonEquals(cosMinusOne, 0)) {
            halfthetaByTanOfHalfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
            halfthetaByTanOfHalfdtheta = -(halfDtheta * transform.getRotation().sin()) / cosMinusOne;
        }

        final Translation translation = transform.getTranslation().rotateBy(new Rotation(halfthetaByTanOfHalfdtheta, -halfDtheta, false));

        return new Twist(translation.x(), translation.y(), dtheta);
    }

    @Override
    public Translation getTranslation() {
        return m_translation;
    }

    @Override
    public Rotation getRotation() {
        return m_rotation;
    }

    /**
     * Transforming this RigidTransform2d means first translating by other.translation and then rotating by
     * other.rotation
     *
     * @param other The other transform.
     * @return This transform * other
     */
    @Override
    public RigidTransform transformBy(final RigidTransform other) {
        return new RigidTransform(
                m_translation.sum(other.m_translation.rotateBy(m_rotation)),
                m_rotation.sum(other.m_rotation));
    }

    public RigidTransform relativeTo(final RigidTransform other) {
        return new RigidTransform(
                m_translation.difference(other.getTranslation()).rotateBy(other.getRotation().inverse()),
                m_rotation.difference(other.getRotation()));
    }

    /**
     * The inverse of this transform "undoes" the effect of translating by this transform.
     *
     * @return The opposite of this transform.
     */
    public RigidTransform inverse() {
        final var invertedRotation = m_rotation.inverse();
        return new RigidTransform(m_translation.inverse().rotateBy(invertedRotation), invertedRotation);
    }

    public RigidTransform normal() {
        return new RigidTransform(m_translation, m_rotation.normal());
    }

    /**
     * Finds the point where the heading of this pose intersects the heading of another. Returns (+INF, +INF) if
     * parallel.
     */
    public Translation intersection(final RigidTransform other) {
        final Rotation otherRotation = other.getRotation();

        if (m_rotation.isParallel(otherRotation)) {
            // Lines are parallel.
            return new Translation(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }

        if (abs(m_rotation.cos()) < abs(otherRotation.cos())) {
            return intersectionInternal(this, other);
        } else {
            return intersectionInternal(other, this);
        }
    }

    /**
     * Return true if this pose is (nearly) colinear with the another.
     */
    public boolean isCollinear(final RigidTransform other) {
        if (!getRotation().isParallel(other.getRotation())) {
            return false;
        }

        final var error = inverse().transformBy(other).toTwist();
        return (MathUtils.epsilonEquals(error.dy, 0.0) && MathUtils.epsilonEquals(error.dtheta, 0.0));
    }

    public boolean epsilonEquals(final RigidTransform other, final double epsilon) {
        return getTranslation().epsilonEquals(other.getTranslation(), epsilon)
                && getRotation().isParallel(other.getRotation());
    }

    private static Translation intersectionInternal(final RigidTransform a, final RigidTransform b) {
        final Rotation aR = a.getRotation();
        final Rotation bR = b.getRotation();
        final Translation aT = a.getTranslation();
        final Translation bT = b.getTranslation();

        final var tanB = bR.tan();
        final var t = ((aT.x() - bT.x()) * tanB + bT.y() - aT.y()) / (aR.sin() - aR.cos() * tanB);

        if (Double.isNaN(t)) {
            return new Translation(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }

        return aT.translateBy(aR.toTranslation().scale(t));
    }

    /**
     * Do twist interpolation of this pose assuming constant curvature.
     */
    @Override
    public RigidTransform interpolate(final RigidTransform other, double x) {
        if (x <= 0) {
            return new RigidTransform(this);
        } else if (x >= 1) {
            return new RigidTransform(other);
        }

        final var error = inverse().transformBy(other).toTwist();
        return transformBy(error.scaled(x).toRigidTransform());
    }

    public final Twist toTwist() {
        return RigidTransform.toTwist(this);
    }

    @Override
    public String toString() {
        return "[T:" + m_translation.toString() + ", R:" + m_rotation.toString() + "]";
    }

    @Override
    public double distance(final RigidTransform other) {
        return inverse().transformBy(other).toTwist().norm();
    }

    @Override
    public boolean equals(final Object other) {
        if (Objects.isNull(other) || !(other instanceof RigidTransform))  {
            return false;
        }

        final var castedOther = ((RigidTransform) other);

        return getRotation().equals(castedOther.getRotation()) && getTranslation().equals(castedOther.getTranslation());
    }

    @Override
    public RigidTransform getRigidTransform() {
        return this;
    }

    @Override
    public RigidTransform mirror() {
        return new RigidTransform(new Translation(getTranslation().x(), -getTranslation().y()), getRotation().inverse());
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_translation, m_rotation);
    }
}
