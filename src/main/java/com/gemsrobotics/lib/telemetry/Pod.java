package com.gemsrobotics.lib.telemetry;

import com.gemsrobotics.lib.telemetry.monitoring.ConnectionMonitor;
import com.gemsrobotics.lib.telemetry.monitoring.Monitor;
import com.gemsrobotics.lib.telemetry.reporting.Reportable;
import com.gemsrobotics.lib.telemetry.reporting.Reporter;
import com.gemsrobotics.lib.telemetry.reporting.Reporter.Event.Kind;
import com.gemsrobotics.lib.utils.FastDoubleToString;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

import java.util.*;

public class Pod {
    public static class Config {
        private Reporter[] m_reporters;
        private Monitor[] m_monitors;

        private Config() {
            m_reporters = null;
            m_monitors = null;
        }

        public Config withReporters(final Reporter... reporters) {
            m_reporters = reporters;
            return this;
        }

        public Config withoutReporters() {
            m_reporters = new Reporter[0];
            return this;
        }

        public Config withMonitors(final Monitor... monitors) {
            m_monitors = monitors;
            return this;
        }

        public Config withoutMonitoring() {
            m_monitors = new Monitor[0];
            return this;
        }

        public void finish() {
            if (isReadyForConstruction()) {
                INSTANCE = Optional.of(new Pod(this));
            } else {
                DriverStation.reportWarning("POD DISABLED", false);
                INSTANCE = Optional.empty();
            }
        }

        private boolean isReadyForConstruction() {
            return !Objects.isNull(m_reporters) && !Objects.isNull(m_monitors);
        }

        public Config withDefaultMonitoring() {
            m_monitors = new Monitor[] {
                    ConnectionMonitor.getInstance()
            };

            return this;
        }
    }

    private static Optional<Pod> INSTANCE;
    private static final String RUNTIME;

    static {
        INSTANCE = Optional.empty();
        RUNTIME = new Date().toString();
    }

    public static Config configure() {
        return new Config();
    }

    public static String getLaunchTime() {
        return RUNTIME;
    }

    private final List<Reporter> m_reportingServices;

    private final Notifier m_thread;

    private Pod(final Config config) {
        m_reportingServices = Arrays.asList(config.m_reporters);

        Arrays.stream(config.m_monitors).forEach(Monitor::doMonitoring);

        m_thread = new Notifier(() -> {
            m_reportingServices.removeIf(Reporter::isHalted);
            m_reportingServices.forEach(Reporter::flush);
        });

        wake();
    }

    public static void wake() {
        INSTANCE.ifPresent(pod -> pod.m_thread.startPeriodic(0.1));
    }

    public static void kill() {
        INSTANCE.ifPresent(pod -> pod.m_thread.stop());
    }

    public static void log(
            final Reporter.Event.Kind kind,
            final Reportable source,
            final String text,
            final Object extra
    ) {
        INSTANCE.ifPresent(pod -> {
            final var event = new Reporter.Event(
                    FastDoubleToString.format(Timer.getFPGATimestamp()),
                    kind,
                    Optional.ofNullable(source).map(Reportable::getName).orElse(null),
                    text,
                    extra
            );

            pod.m_reportingServices.forEach(endpoint -> {
                synchronized (endpoint) {
                    endpoint.push(event);
                }
            });
        });
    }

    public static void catchThrowable(final Object source, final Throwable exception) {
        final var encoded = stackTrace2Strings(exception.getStackTrace());

        if (INSTANCE.isPresent() && !Objects.isNull(source) && !(source instanceof Reporter)) {
            // if the source isn't a reportable, make it one.
            final var verifiedSource = (source instanceof Reportable) ? (Reportable) source : Reportable.makeDummy(source.getClass().getSimpleName());
            log(Kind.ERROR, verifiedSource, exception.getMessage(), encoded);
        } else {
            System.out.println(Arrays.toString(encoded));
        }
    }

    private static String[] stackTrace2Strings(final StackTraceElement[] elements) {
        return Arrays.stream(elements)
                .map(element -> element.getClassName() + "." + element.getMethodName() + "(" + element.getFileName() + ":" + element.getLineNumber() + ")")
                .toArray(String[]::new);
    }
}
