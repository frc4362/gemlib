package com.gemsrobotics.lib.subsystems.drivetrain;

import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Twist;
import com.gemsrobotics.lib.math.interpolation.InterpolatingDouble;
import com.gemsrobotics.lib.data.InterpolatingTreeMap;
import com.gemsrobotics.lib.telemetry.reporting.Reportable;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.Map;

import static com.gemsrobotics.lib.telemetry.reporting.Reporter.Event.Kind.INFO;

/**
 * FieldToVehicleEstimator keeps track of the poses of various coordinate frames throughout the match. A coordinate frame is simply a
 * point and direction in space that defines an (x,y) coordinate system. Transforms (or poses) keep track of the spatial
 * relationship between different frames.
 */
public class FieldToVehicleEstimator implements Loggable, Reportable {
    @Override
    public String configureLogName() {
        return "Odometer";
    }

    private static final int kObservationBufferSize = 100;

    // FPGATimestamp -> RigidTransform2d or Rotation2d
    private transient InterpolatingTreeMap<InterpolatingDouble, RigidTransform> m_fieldToVehicle;
    private Twist m_velocityPredicted, m_velocityMeasured;
    @Log(name="Distance Driven (m)")
    private double m_distanceDriven;

    private final Model m_model;

    public static FieldToVehicleEstimator withStarting(final Model model, final double startTime, final RigidTransform initialPose) {
        final var ret = new FieldToVehicleEstimator(model);
        ret.reset(startTime, initialPose);
        return ret;
    }

    private FieldToVehicleEstimator(final Model model) {
        m_model = model;
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     */
    public synchronized void reset(final double startTime, final RigidTransform initialPose) {
        report(INFO, "Reset @ " + startTime + " with pose " + initialPose.toString());

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
        return getLatestFieldToVehicle().getValue().transformBy(RigidTransform.ofTwist(m_velocityPredicted.scaled(lookaheadTime)));
    }

    public synchronized void addFieldToVehicleObservation(final double timestamp, final RigidTransform observation) {
        m_fieldToVehicle.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addObservation(
            final double timestamp,
            final Twist measuredVelocity,
            final Twist predicted_velocity
    ) {
        final var currentPose = getLatestFieldToVehicle().getValue();
        addFieldToVehicleObservation(timestamp, m_model.solveForwardKinematics(currentPose, measuredVelocity));
        m_velocityMeasured = measuredVelocity;
        m_velocityPredicted = predicted_velocity;
    }

    public synchronized Twist generateOdometryFromSensors(
            final WheelState deltaDistance,
            final Rotation currentRotation
    ) {
        final var lastRotation = getLatestFieldToVehicle().getValue().getRotation();
        final Twist delta = m_model.forwardKinematics(
                lastRotation,
                deltaDistance.left,
                deltaDistance.right,
                currentRotation);

        m_distanceDriven += delta.dx;

        return delta;
    }

    public synchronized double getDistanceDriven() {
        return m_distanceDriven;
    }

    public synchronized Twist getPredictedVelocity() {
        return m_velocityPredicted;
    }

    public synchronized Twist getMeasuredVelocity() {
        return m_velocityMeasured;
    }
}
