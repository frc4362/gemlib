package com.gemsrobotics.lib.structure;

import com.gemsrobotics.lib.telemetry.Pod;
import com.gemsrobotics.lib.telemetry.reporting.Reportable;
import com.gemsrobotics.lib.telemetry.reporting.Reporter.Event.Kind;
import com.gemsrobotics.lib.timing.ElapsedTimer;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.Arrays;
import java.util.List;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;
import static java.lang.Math.floor;

// logs its hz and subsystems
public final class SubsystemManager implements Reportable, Loggable {
    private final double PERIOD = 10 * 1e-3;

	private final ElapsedTimer m_faultCheckTimer;
	private final List<Subsystem> m_subsystems;
    private final Notifier m_updater;
    private final Object m_lock;

    private boolean m_isRunning;
	private double m_lastUpdateTime, m_dt;

	public SubsystemManager(final Subsystem... subsystems) {
		m_subsystems = Arrays.asList(subsystems);
		m_faultCheckTimer = new ElapsedTimer(1.0);
		m_lastUpdateTime = Double.NaN;
		m_isRunning = false;

		m_lock = new Object();

		m_updater = new Notifier(() -> {
            if (!m_isRunning) {
                return;
            }

            synchronized (m_lock) {
                double now = Double.NaN;

                for (final var subsystem : m_subsystems) {
                    now = Timer.getFPGATimestamp();
                    m_dt = now - m_lastUpdateTime;

                    try {
                        subsystem.doPeriodicReads(m_dt);
                    } catch (final Throwable throwable) {
                        Pod.catchThrowable(subsystem, throwable);
                    }

                    if (subsystem.isActive()) {
                        subsystem.onUpdate(now);
                    }
                }

                m_lastUpdateTime = now;

                if (m_faultCheckTimer.hasElapsed()) {
                    m_subsystems.removeIf(subsystem -> {
                        switch (subsystem.checkFaulted()) {
                            case DISABLE_SUBSYSTEM:
                                report(Kind.HARDWARE_FAULT,"Located fault in subsystem \"" + subsystem.getName() + "\".");
                                // Please note the side effects
                                estop(subsystem);
                                return true;
                            case DISABLE_ROBOT:
                                report(Kind.HARDWARE_FAULT, "Located unrecoverable systematic fault in subsystem \"" + subsystem.getName() + "\", KILLING ROBOT.");
                                estop(subsystem);
                                Scheduler.getInstance().disable();
                                disable();
                                return true;
                            case NONE:
                            default:
                                return false;
                        }
                    });
                }
            }
        });
	}

    public synchronized void init() {
	    final int size;

	    synchronized (m_lock) {
            m_lastUpdateTime = getFPGATimestamp();
            m_subsystems.forEach(subsystem -> {
                try {
                    subsystem.onCreate(m_lastUpdateTime);
                } catch (final Throwable throwable) {
                    Pod.catchThrowable(subsystem, throwable);
                }
            });

            size = m_subsystems.size();
        }

		report("Initializing with " + size + " registered subsystems.");
    }

	public synchronized void enable() {
	    if (!m_isRunning) {
            synchronized (m_lock) {
                m_lastUpdateTime = getFPGATimestamp();
                m_subsystems.forEach(subsystem -> {
                    try {
                        subsystem.onEnable(m_lastUpdateTime);
                    } catch (final Throwable throwable) {
                        Pod.catchThrowable(subsystem, throwable);
                    }
                });
            }

            m_updater.startPeriodic(PERIOD);
            m_isRunning = true;

            report("Enabling subsystems.");
        }
    }

    public void disable() {
	    if (m_isRunning) {
            m_updater.stop();

            synchronized (m_lock) {
                m_lastUpdateTime = getFPGATimestamp();
                m_subsystems.forEach(subsystem -> {
                    try {
                        subsystem.onStop(m_lastUpdateTime);
                    } catch (final Throwable throwable) {
                        Pod.catchThrowable(subsystem, throwable);
                    }
                });

                m_isRunning = false;
            }

            report("Disabling subsystems.");
        }
    }

	private void estop(final Subsystem subsystem) {
	    synchronized (subsystem) {
            report(Kind.ERROR, "E-stopping subsystem \"" + subsystem.getName() + "\".");
            subsystem.setActive(false);
            subsystem.onStop(getFPGATimestamp());
            subsystem.setSafeState();
        }
	}

	@Log.VoltageView(
	        name="Subsystem Update Rate (hz)",
	        min=0.0,
            max=100.0,
            orientation="VERTICAL",
            numTicks=10)
	public double hz() {
        return floor(1.0 / m_dt);
    }
}
