package com.gemsrobotics.lib.structure;

import com.gemsrobotics.lib.timing.DeltaTime;
import com.gemsrobotics.lib.timing.ElapsedTimer;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Scheduler;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.Arrays;
import java.util.List;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;
import static java.lang.Math.floor;

// logs its hz and subsystems
public final class SubsystemManager implements Loggable {
    private final double PERIOD = 10 * 1e-3;

	private final ElapsedTimer m_faultCheckTimer;
	private final List<Subsystem> m_subsystems;
    private final Notifier m_updater;
    private final Object m_lock;

    private boolean m_isRunning;
    private double m_dt;
	private DeltaTime m_timer;

	public SubsystemManager(final Subsystem... subsystems) {
		m_subsystems = Arrays.asList(subsystems);
		m_faultCheckTimer = new ElapsedTimer(1.0);
		m_dt = Double.NaN;
		m_timer = new DeltaTime();
		m_isRunning = false;

		m_lock = new Object();

		m_updater = new Notifier(() -> {
            if (!m_isRunning) {
                return;
            }

			synchronized (m_lock) {
				double now = getFPGATimestamp();

                for (final var subsystem : m_subsystems) {
                	// removed this for better state estimation
//                    now = getFPGATimestamp();

                    try {
                        // provide each subsystem the time, it will update the dt internally
                        // this is better than providing the schedule dt as it will be more accurate
                        subsystem.updatePeriodicState(now);
                    } catch (final Throwable throwable) {
						// Pod.catchThrowable(subsystem, throwable);
                    }

                    if (subsystem.isActive()) {
                        subsystem.onUpdate(now);
                    }
                }

                // this is the overall scheduler dt, sum of the subsystems execution times
                m_dt = m_timer.update(now);

                if (m_faultCheckTimer.hasElapsed()) {
                    m_subsystems.removeIf(subsystem -> {
                        switch (subsystem.checkFaulted()) {
                            case DISABLE_SUBSYSTEM:
                                System.out.println("Located fault in subsystem \"" + subsystem.getName() + "\".");
                                // Please note the side effects
                                estop(subsystem);
                                return true;
                            case DISABLE_ROBOT:
								System.out.println("Located unrecoverable systematic fault in subsystem \"" + subsystem.getName() + "\", KILLING ROBOT.");
                                estop(subsystem);
                                Scheduler.getInstance().disable();
                                stop();
                                return true;
                            case NONE:
                            default:
                                return false;
                        }
                    });

                    m_faultCheckTimer.reset();
                }
            }
        });

		m_updater.startPeriodic(PERIOD);
	}

    public synchronized void start() {
	    final int size;

	    if (m_isRunning) {
	    	return;
		}

	    synchronized (m_lock) {
	        m_timer.reset();
            m_subsystems.forEach(subsystem -> {
                try {
                    subsystem.onStart(getFPGATimestamp());
                } catch (final Throwable throwable) {
                    // Pod.catchThrowable(subsystem, throwable);
                }
            });

            size = m_subsystems.size();

			m_isRunning = true;
        }

		System.out.println("Initializing SubsystemManager with " + size + " registered subsystems.");
    }

    public void stop() {
	    if (m_isRunning) {
            m_timer.reset();

            synchronized (m_lock) {
                m_subsystems.forEach(subsystem -> {
                    try {
                        subsystem.onStop(getFPGATimestamp());
                    } catch (final Throwable throwable) {
                        // Pod.catchThrowable(subsystem, throwable);
                    }
                });

                m_isRunning = false;
            }

            System.out.println("Disabling subsystems.");
        }
    }

	private void estop(final Subsystem subsystem) {
		System.out.println("Estopping subsystem \"" + subsystem.getName() + "\".");
        subsystem.setActive(false);
        subsystem.onStop(getFPGATimestamp());
        subsystem.setSafeState();
	}

	@Log.VoltageView(
	        name="Subsystem Update Rate (hz)",
	        min=0,
            max=110,
            orientation="VERTICAL",
            numTicks=10)
	public int hz() {
        return (int) floor(1.0 / m_dt);
    }
}
