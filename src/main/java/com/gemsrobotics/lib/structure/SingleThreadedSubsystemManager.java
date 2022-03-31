package com.gemsrobotics.lib.structure;

import com.gemsrobotics.lib.timing.DeltaTime;
import edu.wpi.first.wpilibj.Timer;

import java.util.List;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;

public final class SingleThreadedSubsystemManager {
	private final List<Subsystem> m_subsystems;
	private boolean m_isStarted;

	public SingleThreadedSubsystemManager(final List<Subsystem> subsystems) {
		m_subsystems = subsystems;
		m_isStarted = false;
	}

	public void update() {
		final double now = Timer.getFPGATimestamp();

		for (final var subsystem : m_subsystems) {
			try {
				// provide each subsystem the time, it will update the dt internally
				// this is better than providing the schedule dt as it will be more accurate
				subsystem.updatePeriodicState(now);
			} catch (final Throwable throwable) {
				System.out.println("ERROR: " + throwable);
			}

			if (subsystem.isActive()) {
				subsystem.onUpdate(now);
			}
		}
	}

	public void start() {
		if (!m_isStarted) {
			m_subsystems.forEach(subsystem -> {
				try {
					subsystem.onStart(getFPGATimestamp());
				} catch (final Throwable throwable) {
					System.out.println("ERROR: " + throwable);
				}
			});
		}

		m_isStarted = true;
	}

	public void stop() {

		m_isStarted = false;
		m_subsystems.forEach(subsystem -> {
			try {
				subsystem.onStop(getFPGATimestamp());
			} catch (final Throwable throwable) {
				System.out.println("ERROR: " + throwable);
			}
		});
	}
}
