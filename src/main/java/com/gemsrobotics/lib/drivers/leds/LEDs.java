package com.gemsrobotics.lib.drivers.leds;

import edu.wpi.first.wpilibj.Timer;

import java.awt.Color;

import static java.lang.Math.abs;
import static java.lang.Math.sin;

@SuppressWarnings("WeakerAccess")
public abstract class LEDs {
    private boolean m_on = false;
    private float m_rainbowAccumulator = 0.0f;

    public abstract void setColor(Color color, float brightness);

    public final void setOn(final boolean on) {
        m_on = on;
    }

    public final boolean isOn() {
        return m_on;
    }

	public final void setRainbow(final float wavelength, final float intensity) {
		m_rainbowAccumulator += (1 / abs(wavelength)) * 0.02;

		setColor(Color.getHSBColor(m_rainbowAccumulator * 360f, 1, 1), intensity);
	}

    protected final float calculatePulseIntensity(final float wavelengthSeconds) {
        return (float) (sin(Timer.getFPGATimestamp() / wavelengthSeconds) + 1) / 2;
    }

    public final void setColorWave(final Color color, final float wavelength) {
        setColor(color, calculatePulseIntensity(wavelength));
    }
}
