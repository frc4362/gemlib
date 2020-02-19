package com.gemsrobotics.lib.data;

public final class RollingAverageDouble extends LimitedQueue<Double> {
    public RollingAverageDouble(final int sampleSize) {
        super(sampleSize);
    }

    public double getAverage() {
        return stream().mapToDouble(Double::valueOf).average().orElse(0.0);
    }
}
