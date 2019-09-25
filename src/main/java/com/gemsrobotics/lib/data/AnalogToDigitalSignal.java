package com.gemsrobotics.lib.data;

import java.util.function.Function;

public class AnalogToDigitalSignal extends DigitalSignalTrigger {
    private final Function<Void, Boolean> m_operation;

    public AnalogToDigitalSignal(final Function<Void, Double> getter, final Function<Double, Boolean> encoder) {
        m_operation = getter.andThen(encoder);
    }

    @Override
    public final boolean get() {
        return m_operation.apply(null);
    }
}
