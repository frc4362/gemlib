package com.gemsrobotics.lib.timing;

import edu.wpi.first.wpilibj.Timer;

public class DeltaTime {
    private double m_lastTime;

    public DeltaTime() {
        m_lastTime = Double.NaN;
    }

    public double update() {
        final double now = Timer.getFPGATimestamp();

        if (Double.isNaN(m_lastTime)) {
            m_lastTime = now;
        }

        final var dt = now - m_lastTime;
        m_lastTime = now;
        return dt;
    }
}
