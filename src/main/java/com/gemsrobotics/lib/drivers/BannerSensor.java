package com.gemsrobotics.lib.drivers;

import edu.wpi.first.wpilibj.AnalogInput;

import java.util.LinkedHashMap;
import java.util.function.Supplier;

public final class BannerSensor {
    private static final double DEFAULT_THRESHOLD = 0.1;

    private final AnalogInput m_input;
    private final double m_threshold;

    public BannerSensor(final int port, final double threshold) {
        m_input = new AnalogInput(port);
        m_threshold = threshold;
    }

    public BannerSensor(final int port) {
        this(port, DEFAULT_THRESHOLD);
    }

    public AnalogInput getRawSensor() {
        return m_input;
    }

    public boolean get() {
        return m_input.getAverageVoltage() > m_threshold;
    }
}
