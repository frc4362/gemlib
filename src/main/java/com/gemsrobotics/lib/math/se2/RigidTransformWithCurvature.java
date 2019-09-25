package com.gemsrobotics.lib.math.se2;

import com.gemsrobotics.lib.utils.FastDoubleToString;
import com.gemsrobotics.lib.utils.MathUtils;
import com.google.gson.annotations.SerializedName;

import java.util.Objects;

@SuppressWarnings({"unused", "WeakerAccess"})
public class RigidTransformWithCurvature implements
        IRigidTransform2d<RigidTransformWithCurvature>,
        ICurvature<RigidTransformWithCurvature>
{
    public static RigidTransformWithCurvature identity() {
        return new RigidTransformWithCurvature();
    }

    @SerializedName("pose")
    protected final RigidTransform m_pose;
    @SerializedName("curvature")
    protected final double m_curvature;
    @SerializedName("dcurvature_ds")
    protected final double m_dcurvatureDs;

    public RigidTransformWithCurvature(final RigidTransform pose, final double curvature, final double dcurvatureDs) {
        m_pose = pose;
        m_curvature = curvature;
        m_dcurvatureDs = dcurvatureDs;
    }

    public RigidTransformWithCurvature() {
        this(new RigidTransform(), 0.0, 0.0);
    }

    public RigidTransformWithCurvature(final RigidTransform pose, final double curvature) {
        this(pose, curvature, 0.0);
    }

    public RigidTransformWithCurvature(final Translation translation, final Rotation rotation, final double curvature) {
        this(new RigidTransform(translation, rotation), curvature, 0.0);
    }

    public RigidTransformWithCurvature(
            final Translation translation,
            final Rotation rotation,
            final double curvature,
            final double dcurvatureDs
    ) {
        this(new RigidTransform(translation, rotation), curvature, dcurvatureDs);
    }

    @Override
    public final RigidTransform getRigidTransform() {
        return m_pose;
    }

    @Override
    public RigidTransformWithCurvature transformBy(final RigidTransform other) {
        return new RigidTransformWithCurvature(
                getRigidTransform().transformBy(other),
                getCurvature(),
                getDCurvatureDs());
    }

    @Override
    public RigidTransformWithCurvature mirror() {
        return new RigidTransformWithCurvature(
                getRigidTransform().mirror().getRigidTransform(),
                -getCurvature(),
                -getDCurvatureDs());
    }

    @Override
    public double getCurvature() {
        return m_curvature;
    }

    @Override
    public double getDCurvatureDs() {
        return m_dcurvatureDs;
    }

    @Override
    public final Translation getTranslation() {
        return getRigidTransform().getTranslation();
    }

    @Override
    public final Rotation getRotation() {
        return getRigidTransform().getRotation();
    }

    @Override
    public RigidTransformWithCurvature interpolate(final RigidTransformWithCurvature other, final double x) {
        return new RigidTransformWithCurvature(getRigidTransform().interpolate(other.getRigidTransform(), x),
                MathUtils.lerp(getCurvature(), other.getCurvature(), x),
                MathUtils.lerp(getDCurvatureDs(), other.getDCurvatureDs(), x));
    }

    @Override
    public double distance(final RigidTransformWithCurvature other) {
        return getRigidTransform().distance(other.getRigidTransform());
    }

    @Override
    public boolean equals(final Object other) {
        if (Objects.isNull(other) || !(other instanceof RigidTransformWithCurvature)) {
            return false;
        }

        final var rt2dwc = (RigidTransformWithCurvature) other;

        return getRigidTransform().equals(rt2dwc.getRigidTransform())
                   && MathUtils.epsilonEquals(getCurvature(), rt2dwc.getCurvature())
                   && MathUtils.epsilonEquals(getDCurvatureDs(), rt2dwc.getDCurvatureDs());
    }

    @Override
    public String toString() {
        return "[" + getRigidTransform().toString()
                       + ", curvature: " + FastDoubleToString.format(getCurvature(), 3)
                       + ", dcurvature_ds: " + FastDoubleToString.format(getDCurvatureDs(), 3) + "]";
    }
}
