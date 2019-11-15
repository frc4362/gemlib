package com.gemsrobotics.lib.data;

import java.util.function.Supplier;

public final class CachedBoolean extends CachedValue<Boolean> {
    public CachedBoolean(final double timeout, final Supplier<Boolean> source) {
        super(Boolean.class, timeout, source);
    }

    // rising edge since last get, not since last update period
    public boolean getRisingEdge() {
        return get() && !getLastValue().orElse(false);
    }

    // falling edge since last get, not since last update period
    public boolean getFallingEdge() {
        return !get() && getLastValue().orElse(false);
    }
}
