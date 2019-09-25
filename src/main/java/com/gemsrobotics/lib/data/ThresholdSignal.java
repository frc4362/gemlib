package com.gemsrobotics.lib.data;

import java.util.function.Supplier;

public class ThresholdSignal extends AnalogToDigitalSignal {
    public ThresholdSignal(final Supplier<Double> getter, final double threshold) {
        super(t -> getter.get(), n -> n > threshold);
    }
}
