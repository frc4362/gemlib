package com.gemsrobotics.lib.subsystems.drivetrain;

import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Twist;
import com.gemsrobotics.lib.math.interpolation.InterpolatingDouble;
import com.gemsrobotics.lib.data.InterpolatingTreeMap;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.Map;

import static java.lang.Math.abs;

/**
 * FieldToVehicleEstimator keeps track of the poses of various coordinate frames throughout the match. A coordinate frame is simply a
 * point and direction in space that defines an (x,y) coordinate system. Transforms (or poses) keep track of the spatial
 * relationship between different frames.
 */
public class FieldToVehicleEstimator implements Loggable {
    @Override
    public String configureLogName() {
        return "Odometer";
    }

    private static final int kObservationBufferSize = 100;

    // FPGATimestamp -> RigidTransform
    private transient InterpolatingTreeMap<InterpolatingDouble, RigidTransform> m_fieldToVehicle;
    private Twist m_velocityPredicted, m_velocityMeasured;
    @Log(name="Distance Driven (m)")
    private double m_distanceDriven;

    private final DifferentialDriveModel m_model;

    public static FieldToVehicleEstimator withStarting(
            final DifferentialDriveModel model,
            final double startTime,
            final RigidTransform initialPose
    ) {
        final var ret = new FieldToVehicleEstimator(model);
        ret.reset(startTime, initialPose);
        return ret;
    }

    private FieldToVehicleEstimator(final DifferentialDriveModel model) {
        m_model = model;
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     */
    public synchronized void reset(final double startTime, final RigidTransform initialPose) {
        m_fieldToVehicle = new InterpolatingTreeMap<>(kObservationBufferSize);
        m_fieldToVehicle.put(new InterpolatingDouble(startTime), initialPose);

        m_velocityPredicted = Twist.identity();
        m_velocityMeasured = Twist.identity();

        resetDistanceDriven();
    }

    public synchronized void resetDistanceDriven() {
        m_distanceDriven = 0.0;
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
     * to fill in the gaps.
     */
    public synchronized RigidTransform getFieldToVehicle(final double timestamp) {
        return m_fieldToVehicle.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, RigidTransform> getLatestFieldToVehicle() {
        return m_fieldToVehicle.lastEntry();
    }

    @Log.ToString(name="Estimated Field to Vehicle (Pose)")
    public synchronized RigidTransform getLatestFieldToVehicleValue() {
        return getLatestFieldToVehicle().getValue();
    }

    public synchronized RigidTransform getPredictedFieldToVehicle(final double lookaheadTime) {
        return getLatestFieldToVehicle().getValue().transformBy(m_velocityPredicted.scaled(lookaheadTime).toRigidTransform());
    }

    public synchronized void addFieldToVehicleObservation(final double timestamp, final RigidTransform observation) {
        m_fieldToVehicle.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addObservation(
            final double timestamp,
            final WheelState deltaDistance,
            final Rotation currentRotation,
            final Twist velocityPredicted
    ) {
        final var latestPose = getLatestFieldToVehicle().getValue();
        final Twist displacement = m_model.forwardKinematics(
                latestPose.getRotation(),
                deltaDistance.left,
                deltaDistance.right,
                currentRotation);

        addFieldToVehicleObservation(timestamp, m_model.solveForwardKinematics(latestPose, displacement));

        m_velocityMeasured = displacement;
        m_velocityPredicted = velocityPredicted;

        m_distanceDriven += abs(displacement.dx);
    }

    public synchronized double getDistanceDriven() {
        return m_distanceDriven;
    }
}
