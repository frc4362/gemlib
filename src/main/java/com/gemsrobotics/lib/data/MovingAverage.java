package com.gemsrobotics.lib.data;

public final class MovingAverage extends LimitedQueue<Double> {
    public MovingAverage(final int sampleSize) {
        super(sampleSize);
    }

    public double getAverage() {
        return stream().mapToDouble(Double::valueOf).sum();
    }
}
