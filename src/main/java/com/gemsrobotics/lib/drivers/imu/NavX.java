package com.gemsrobotics.lib.drivers.imu;

import com.gemsrobotics.lib.math.se2.Rotation;
import com.kauailabs.navx.AHRSProtocol;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.ITimestampedDataSubscriber;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

import static java.lang.Math.abs;

public class NavX implements CollisionDetectingIMU {
    public static final double
            COLLISION_JERK_THRESHOLD = 950,
            TIPPING_THRESHOLD_DEGREES = 11;

    private class PeriodicIO implements ITimestampedDataSubscriber {
        @Override
        public void timestampedDataReceived(
                final long systemTimestamp,
                final long sensorTimestamp,
                final AHRSProtocol.AHRSUpdateBase update,
                final Object context
        ) {
            synchronized (NavX.this) {
                // This handles the fact that the sensor is inverted from our coordinate conventions.
                if (m_lastSensorTimestamp != kInvalidTimestamp && m_lastSensorTimestamp < sensorTimestamp) {
                    m_yawRateDegreesPerSecond = 1000.0 * (-m_yawDegrees - update.yaw) / (double) (sensorTimestamp - m_lastSensorTimestamp);
                }

                m_lastSensorTimestamp = sensorTimestamp;
                m_yawDegrees = -update.yaw;
                m_fusedHeading = -update.fused_heading;
            }
        }
    }

    protected AHRS m_ahrs;

    protected Rotation m_angleAdjustment;

    protected double m_yawDegrees;
    protected double m_fusedHeading;
    protected double m_yawRateDegreesPerSecond;
    protected final long kInvalidTimestamp = -1;
    protected long m_lastSensorTimestamp;

    protected double m_prevAccelX;
    protected double m_prevAccelY;
    protected double m_prevTime;

    protected double m_collisionJerkThreshold;
    protected double m_tippingThresholdDegrees;

    public NavX(final SPI.Port spiPort) {
        m_angleAdjustment = Rotation.identity();
        m_prevAccelX = 0;
        m_prevAccelY = 0;
        m_prevTime = 0;
        m_collisionJerkThreshold = COLLISION_JERK_THRESHOLD;
        m_tippingThresholdDegrees = TIPPING_THRESHOLD_DEGREES;

        m_ahrs = new AHRS(spiPort, (byte) 200);

        resetState();

        m_ahrs.registerCallback(new PeriodicIO(), null);
    }

    public NavX() {
        this(SPI.Port.kMXP);
    }

    public boolean isPresent() {
        return m_ahrs.isConnected();
    }

    public synchronized boolean reset() {
        m_ahrs.reset();
        resetState();
        return true;
    }

    public synchronized void zeroYaw() {
        m_ahrs.zeroYaw();
        resetState();
    }

    private void resetState() {
        m_lastSensorTimestamp = kInvalidTimestamp;
        m_yawDegrees = 0.0;
        m_yawRateDegreesPerSecond = 0.0;
    }

    public synchronized void setAngleAdjustment(Rotation adjustment) {
        m_angleAdjustment = adjustment;
    }

    public synchronized double getRawYawDegrees() {
        return m_yawDegrees;
    }

    public Rotation getYaw() {
        return m_angleAdjustment.rotateBy(Rotation.degrees(getRawYawDegrees()));
    }

    public Rotation getRoll() {
        return Rotation.degrees(m_ahrs.getRoll());
    }

    public Rotation getPitch() {
        return Rotation.degrees(m_ahrs.getPitch());
    }

    public double getYawRateDegreesPerSec() {
        return m_yawRateDegreesPerSecond;
    }

    public double getFusedHeading() {
        return m_fusedHeading;
    }

    public double getYawRateRadiansPerSec() {
        return 180.0 / Math.PI * getYawRateDegreesPerSec();
    }

    public double getRawAccelX() {
        return m_ahrs.getRawAccelX();
    }

    public double getRawAccelY() {
        return m_ahrs.getRawAccelY();
    }

    public double getRawAccelZ() {
        return m_ahrs.getRawAccelZ();
    }

    public synchronized void setThresholds(final Thresholds thresholds) {
        setCollisionJerkThreshold(thresholds.collisionJerkThreshold);
        setTippingThreshold(thresholds.tipThresholdDegrees);
    }

    public synchronized void setCollisionJerkThreshold(double jerkCollisionThreshold) {
        m_collisionJerkThreshold = jerkCollisionThreshold;
    }

    public synchronized void setTippingThreshold(double tippingThreshold) {
        m_tippingThresholdDegrees = tippingThreshold;
    }

    public boolean isCollisionOccurring() {
        boolean collisionOccurring = false;

        final double accelX = m_ahrs.getWorldLinearAccelX();
        final double accelY = m_ahrs.getWorldLinearAccelY();

        final double currTime = Timer.getFPGATimestamp();
        final double dt = currTime - m_prevTime;

        final double jerkX = (accelX - m_prevAccelX) / dt;
        final double jerkY = (accelY - m_prevAccelY) / dt;

        if (abs(jerkX) > m_collisionJerkThreshold || abs(jerkY) > m_collisionJerkThreshold) {
            collisionOccurring = true;
        }

        m_prevAccelX = accelX;
        m_prevAccelY = accelY;

        if (m_prevTime == 0) {
            m_prevTime = currTime;
            return false;
        }

        m_prevTime = currTime;

        return collisionOccurring;
    }

    public boolean isTipping() {
        return abs(m_ahrs.getPitch()) > m_tippingThresholdDegrees || abs(m_ahrs.getRoll()) > m_tippingThresholdDegrees;
    }
}

