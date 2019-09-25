package com.gemsrobotics.lib.drivers.transmission;

public class SingleSpeedTransmission implements Transmission {
    @Override
    public void setHighGear(boolean useHighGear) {
        // nope
    }

    @Override
    public boolean isHighGear() {
        return false;
    }
}
