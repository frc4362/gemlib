package com.gemsrobotics.lib.drivers;

import edu.wpi.first.wpilibj.DoubleSolenoid;

import java.util.Objects;

public class LazyDoubleSolenoid {
	private final DoubleSolenoid m_base;
	private DoubleSolenoid.Value m_last;

	public LazyDoubleSolenoid(final DoubleSolenoid solenoid) {
		m_base = solenoid;
		m_last = null;
	}

	public void set(final DoubleSolenoid.Value val) {
		if (Objects.isNull(m_last) || val != m_last) {
			m_base.set(val);
			m_last = val;
		}
	}
}
