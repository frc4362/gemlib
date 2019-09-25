package com.gemsrobotics.lib.drivers.imu;

import com.gemsrobotics.lib.math.se2.Rotation;

public interface IMU {
    Rotation getYaw();
    Rotation getPitch();
    Rotation getRoll();

    boolean isPresent();
    boolean reset();
}
