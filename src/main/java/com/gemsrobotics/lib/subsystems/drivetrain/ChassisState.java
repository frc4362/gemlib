package com.gemsrobotics.lib.subsystems.drivetrain;

import com.gemsrobotics.lib.utils.FastDoubleToString;

// Can refer to velocity or acceleration depending on context.
public class ChassisState {
    public double linearMeters, angularRadians;

    public ChassisState(final double linear, final double angular) {
        this.linearMeters = linear;
        this.angularRadians = angular;
    }

    public ChassisState() {
        this(0, 0);
    }

    @Override
    public String toString() {
        return "[vx: " + FastDoubleToString.format(linearMeters, 3) + ", omega: " + FastDoubleToString.format(angularRadians, 3) + "]";
    }
}
