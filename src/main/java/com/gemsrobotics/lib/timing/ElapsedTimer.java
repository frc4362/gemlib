package com.gemsrobotics.lib.timing;

import edu.wpi.first.wpilibj.Timer;

public class ElapsedTimer {
	private final double m_duration;

	private double m_startTime;

	public ElapsedTimer(final double duration) {
		m_duration = duration;
		reset();
	}

	public synchronized void reset() {
		m_startTime = Timer.getFPGATimestamp();
	}

	public boolean hasElapsed() {
		return (Timer.getFPGATimestamp() - m_startTime) > m_duration;
	}

	public double getDuration() {
		return m_duration;
	}
}
