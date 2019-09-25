package com.gemsrobotics.lib.drivers.imu;

public interface CollisionDetectingIMU extends IMU {
    boolean isCollisionOccurring();
    boolean isTipping();

    class Thresholds {
        double collisionJerkThreshold;
        double tipThresholdDegrees;
    }
}
