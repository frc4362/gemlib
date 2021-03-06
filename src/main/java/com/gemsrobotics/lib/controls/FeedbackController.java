package com.gemsrobotics.lib.controls;

public abstract class FeedbackController {
    public abstract void reset();
    public abstract double update(double dt, double input);

    public abstract void setReference(double reference);
    public abstract double getReference();

    public abstract void setTolerance(double tolerance);
    public abstract double getTolerance();

    public abstract boolean atReference(double tolerance);
    public final boolean atReference() {
        return atReference(getTolerance());
    }
}
