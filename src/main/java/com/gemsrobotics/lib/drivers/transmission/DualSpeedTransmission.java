package com.gemsrobotics.lib.drivers.transmission;

import edu.wpi.first.wpilibj.Solenoid;

public class DualSpeedTransmission implements Transmission {
    protected final Solenoid m_solenoid;
    protected boolean m_inverted;

    /**
     * By default, the transmission is in low gear when unpowered
     * @param port The port of the single solenoid to use to shit
     */
    public DualSpeedTransmission(final int port) {
        m_solenoid = new Solenoid(port);
        m_inverted = false;
    }

    /**
     * Swaps the shifter to be in high gear when unpowered, and low gear when powered.
     */
    public void setInverted(final boolean inverted) {
        m_inverted = inverted;
    }

    public boolean isInverted() {
        return m_inverted;
    }

    @Override
    public void setHighGear(final boolean useHighGear) {
        m_solenoid.set(useHighGear ^ m_inverted);
    }

    @Override
    public boolean isHighGear() {
        return m_solenoid.get() ^ m_inverted;
    }
}
