package com.gemsrobotics.lib.drivers.imu;

import com.gemsrobotics.lib.math.se2.Rotation;
import com.kauailabs.navx.AHRSProtocol;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.ITimestampedDataSubscriber;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

import static java.lang.Math.*;

public final class NavX {
    private static final byte
            UPDATE_RATE_HZ = (byte) 200;
    public static final double
            COLLISION_JERK_THRESHOLD = 950,
            TIPPING_THRESHOLD_DEGREES = 11;
    private static final long
            INVALID_TIMESTAMP = -1L;

    // This handles the fact that the sensor is inverted from our coordinate conventions.
    private class PeriodicIO implements ITimestampedDataSubscriber {
        @Override
        public void timestampedDataReceived(
                final long systemTimestamp,
                final long sensorTimestamp,
                final AHRSProtocol.AHRSUpdateBase update,
                final Object context
        ) {
            synchronized (NavX.this) {
                if (m_lastSensorTimestamp != INVALID_TIMESTAMP && m_lastSensorTimestamp < sensorTimestamp) {
                    m_yawRateDegreesPerSecond = 1000.0 * (-m_yawDegrees - update.yaw) / (double) (sensorTimestamp - m_lastSensorTimestamp);
                }

                m_lastSensorTimestamp = sensorTimestamp;
                m_yawDegrees = -update.yaw;
                m_fusedHeading = -update.fused_heading;
            }
        }
    }

    private final AHRS m_ahrs;

    private Rotation m_yawOffset;
    private double m_yawDegrees;
    private double m_fusedHeading;
    private double m_yawRateDegreesPerSecond;
    private long m_lastSensorTimestamp;
    private double m_lastAccelerationX;
    private double m_lastAccelerationY;
    private double m_lastTime;

    public NavX(final SPI.Port spiPort) {
        m_lastSensorTimestamp = INVALID_TIMESTAMP;
        m_yawOffset = Rotation.identity();
        m_lastAccelerationX = 0;
        m_lastAccelerationY = 0;
        m_lastTime = 0;

        m_ahrs = new AHRS(spiPort, UPDATE_RATE_HZ);

        resetState();

        m_ahrs.registerCallback(new PeriodicIO(), null);
    }

    public NavX() {
        this(SPI.Port.kMXP);
    }

    public synchronized boolean isPresent() {
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

    private synchronized void resetState() {
        m_lastSensorTimestamp = INVALID_TIMESTAMP;
        m_yawDegrees = 0.0;
        m_yawRateDegreesPerSecond = 0.0;
    }

    public synchronized void setAngleAdjustment(final Rotation adjustment) {
        m_yawOffset = adjustment;
    }

    public synchronized double getRawYawDegrees() {
        return m_yawDegrees;
    }

    public synchronized Rotation getYaw() {
        return m_yawOffset.rotateBy(Rotation.degrees(getRawYawDegrees()));
    }

    public synchronized Rotation getRoll() {
        return Rotation.degrees(m_ahrs.getRoll());
    }

    public synchronized Rotation getPitch() {
        return Rotation.degrees(m_ahrs.getPitch());
    }

    public synchronized double getYawRateDegreesPerSec() {
        return m_yawRateDegreesPerSecond;
    }

    public synchronized Rotation getFusedHeading() {
        return Rotation.degrees(m_fusedHeading);
    }

    public synchronized double getYawRateRadiansPerSec() {
        return toRadians(getYawRateDegreesPerSec());
    }

    public synchronized double getRawAccelerationX() {
        return m_ahrs.getRawAccelX();
    }

    public synchronized double getRawAccelerationY() {
        return m_ahrs.getRawAccelY();
    }

    public synchronized double getRawAccelerationZ() {
        return m_ahrs.getRawAccelZ();
    }

    public final synchronized boolean isCollisionOccurring() {
        boolean isCollisionOccurring = false;

        final double accelerationX = m_ahrs.getWorldLinearAccelX();
        final double accelerationY = m_ahrs.getWorldLinearAccelY();

        final double t = Timer.getFPGATimestamp();
        final double dt = t - m_lastTime;

        final double jerkX = (accelerationX - m_lastAccelerationX) / dt;
        final double jerkY = (accelerationY - m_lastAccelerationY) / dt;

        if (hypot(jerkX, jerkY) > COLLISION_JERK_THRESHOLD) {
            isCollisionOccurring = true;
        }

        m_lastAccelerationX = accelerationX;
        m_lastAccelerationY = accelerationY;

        if (m_lastTime == 0) {
            m_lastTime = t;
            return false;
        }

        m_lastTime = t;

        return isCollisionOccurring;
    }

    public final synchronized boolean isTipping() {
        return abs(m_ahrs.getPitch()) > TIPPING_THRESHOLD_DEGREES || abs(m_ahrs.getRoll()) > TIPPING_THRESHOLD_DEGREES;
    }
}
