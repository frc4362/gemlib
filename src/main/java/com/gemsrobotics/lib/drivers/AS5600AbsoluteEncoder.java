package com.gemsrobotics.lib.drivers;

import com.ctre.phoenix.motorcontrol.SensorCollection;

import java.util.Optional;

import static java.lang.Integer.min;

public final class AS5600AbsoluteEncoder {
	private final SensorCollection m_sensors;
	private Optional<Integer> m_lastValue;

	public AS5600AbsoluteEncoder(final SensorCollection sensors) {
		m_sensors = sensors;
		m_lastValue = Optional.empty();
	}

	public Optional<Integer> getPosition() {
		final int raw = m_sensors.getPulseWidthRiseToFallUs();

		if (raw == 0) {
			return m_lastValue;
		}

		final var actual = Optional.of(min(4096, raw - 128));
		m_lastValue = actual;
		return actual;
	}
}
