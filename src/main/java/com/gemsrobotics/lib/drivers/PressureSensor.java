package com.gemsrobotics.lib.drivers;

import edu.wpi.first.wpilibj.AnalogInput;

// http://www.revrobotics.com/wp-content/uploads/2015/11/REV-11-1107-DS-00.pdf
public final class PressureSensor {
    private final AnalogInput m_input;

    public PressureSensor(final int port) {
        m_input = new AnalogInput(port);
    }

    public double getPSI() {
        return (250.0 * m_input.getVoltage() / 5.0) - 25.0;
    }
}
