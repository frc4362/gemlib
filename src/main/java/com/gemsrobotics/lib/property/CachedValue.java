package com.gemsrobotics.lib.property;

import com.gemsrobotics.lib.telemetry.reporting.Reporter;
import com.gemsrobotics.lib.telemetry.reporting.Reportable;
import com.gemsrobotics.lib.timing.ElapsedTimer;

import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

public class CachedValue<T> implements Reportable {
	@Override
	public String getName() {
		return "CachedValue-" + m_type;
	}

	private final Class<T> m_type;
	private final ElapsedTimer m_timer;
	private final Supplier<T> m_source;

	private boolean m_initialized;
	private T m_value, m_oldValue;

	public CachedValue(
			final Class<T> type,
			final double timeout,
			final Supplier<T> source
	) {
		m_type = type;
		m_timer = new ElapsedTimer(timeout);
		m_timer.reset();
		m_source = source;

		m_initialized = false;
		m_value = null;
		m_oldValue = null;
	}

	protected final synchronized void set(final T candidate) {
		m_oldValue = m_value;
		m_value = candidate;
	}

	public final synchronized T get() {
		try {
			if (!m_initialized) {
				set(m_source.get());
				m_initialized = true;
			}

			if (m_timer.hasElapsed()) {
				set(m_source.get());
				m_timer.reset();
			}
		} catch (final Exception ex) {
			report(Reporter.Event.Kind.ERROR, "Cached value unable to be retrieved: " + ex.getMessage());
			return m_oldValue;
		}

		return m_value;
	}

	public boolean hasExpired() {
	    return m_timer.hasElapsed();
    }

	public Optional<T> getLastValue() {
		return Optional.ofNullable(m_oldValue);
	}

	public <U> CachedValue<U> map(final Class<U> newType, final Function<T, U> f) {
		return new CachedValue<>(newType, m_timer.getDuration(), () -> f.apply(m_source.get()));
	}
}
