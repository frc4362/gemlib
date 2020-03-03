package com.gemsrobotics.lib.drivers.leds;

import com.ctre.phoenix.CANifier;

import java.awt.Color;

public class LEDCanifier extends LEDs {
	private final CANifier m_canifier;

    private Color m_colorLast;
	private float m_intensityLast;

	public LEDCanifier(final CANifier canifier) {
		m_canifier = canifier;
		m_colorLast = Color.BLACK;
		m_intensityLast = Float.NaN;
	}

	@Override
	public synchronized void setColor(Color color, float intensity) {
	    if (!isOn()) {
	        color = Color.BLACK;
	        intensity = 0.0f;
        }

		if (!color.equals(m_colorLast) || intensity != m_intensityLast) {
			m_colorLast = color;
			m_intensityLast = intensity;

			m_canifier.setLEDOutput((color.getRed() / 255.0f) * intensity, CANifier.LEDChannel.LEDChannelC);
			m_canifier.setLEDOutput((color.getGreen() / 255.0f) * intensity, CANifier.LEDChannel.LEDChannelA);
			m_canifier.setLEDOutput((color.getBlue() / 255.0f) * intensity, CANifier.LEDChannel.LEDChannelB);
		}
	}
}
