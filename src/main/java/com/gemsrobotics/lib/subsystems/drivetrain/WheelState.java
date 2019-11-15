package com.gemsrobotics.lib.subsystems.drivetrain;

import com.gemsrobotics.lib.utils.FastDoubleToString;

import java.util.function.DoubleFunction;

// Can refer to velocity, acceleration, torque, voltage, etcetera, depending on context.
public class WheelState {
    public double left, right;

    public WheelState(final double left, final double right) {
        this.left = left;
        this.right = right;
    }

    public WheelState() {
        this(0, 0);
    }

    public WheelState(final WheelState other) {
        this(other.left, other.right);
    }

    public WheelState copy() {
        return new WheelState(left, right);
    }

    public WheelState map(final DoubleFunction<Double> f) {
        return new WheelState(f.apply(left), f.apply(right));
    }

    public WheelState inverse() {
        return map(n -> -n);
    }

    public WheelState sum(final WheelState other) {
        return new WheelState(left + other.left, right + other.right);
    }

    public WheelState difference(final WheelState other) {
        return sum(other.inverse());
    }

    public void flip() {
        final var tempL = left;
        left = -right;
        right = -tempL;
    }

    @Override
    public String toString() {
        return "[" + FastDoubleToString.format(left, 2) + ", " + FastDoubleToString.format(right, 2) + "]";
    }
}
