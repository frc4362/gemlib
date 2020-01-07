package com.gemsrobotics.lib.subsystems.drivetrain;

import com.gemsrobotics.lib.utils.FastDoubleToString;

// Can refer to velocity or acceleration depending on context.
public class ChassisState {
    public double linear, angular;

    public ChassisState(final double linear, final double angular) {
        this.linear = linear;
        this.angular = angular;
    }

    public ChassisState() {
        this(0, 0);
    }

    @Override
    public String toString() {
        return "[vx: " + FastDoubleToString.format(linear, 3) + ", omega: " + FastDoubleToString.format(angular, 3) + "]";
    }

    public String toAccelerationString() {
        return "[vx^2: " + FastDoubleToString.format(linear, 3) + ", omega^2: " + FastDoubleToString.format(angular, 3) + "]";
    }
}
