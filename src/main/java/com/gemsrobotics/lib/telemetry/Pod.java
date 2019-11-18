package com.gemsrobotics.lib.telemetry;

import com.gemsrobotics.lib.telemetry.monitoring.ConnectionMonitor;
import com.gemsrobotics.lib.telemetry.monitoring.Monitor;
import com.gemsrobotics.lib.telemetry.reporting.Reportable;
import com.gemsrobotics.lib.telemetry.reporting.ReportingEndpoint;
import com.gemsrobotics.lib.telemetry.reporting.ReportingEndpoint.Event.Kind;
import com.gemsrobotics.lib.utils.FastDoubleToString;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

import java.util.*;

public class Pod {
    public static class Config {
        private ReportingEndpoint[] m_reportingEndpoints;
        private Monitor[] m_monitors;

        private Config() {
            m_reportingEndpoints = null;
            m_monitors = null;
        }

        public Config withReporters(final ReportingEndpoint... endpoints) {
            m_reportingEndpoints = endpoints;
            return this;
        }

        public Config withoutReporters() {
            m_reportingEndpoints = new ReportingEndpoint[0];
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

        public void wake() {
            if (isReadyForConstruction()) {
                INSTANCE = Optional.of(new Pod(this));
            } else {
                DriverStation.reportWarning("POD DISABLED", false);
                INSTANCE = Optional.empty();
            }
        }

        private boolean isReadyForConstruction() {
            return !Objects.isNull(m_reportingEndpoints) && !Objects.isNull(m_monitors);
        }

        public Config withDefaultMonitoring() {
            withMonitors(ConnectionMonitor.getInstance());
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

    private final List<ReportingEndpoint> m_reportingServices;

    private final Notifier m_thread;

    private Pod(final Config config) {
        m_reportingServices = Arrays.asList(config.m_reportingEndpoints);

        Arrays.stream(config.m_monitors).forEach(Monitor::doMonitoring);

        m_thread = new Notifier(() -> {
            m_reportingServices.removeIf(ReportingEndpoint::isHalted);
            m_reportingServices.forEach(ReportingEndpoint::flush);
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
            final ReportingEndpoint.Event.Kind kind,
            final Reportable source,
            final String text,
            final Object extra
    ) {
        INSTANCE.ifPresent(pod -> {
            final var event = new ReportingEndpoint.Event(
                    FastDoubleToString.format(Timer.getFPGATimestamp()),
                    kind,
                    Optional.ofNullable(source).map(Reportable::getName).orElse(null),
                    text,
                    extra
            );

            pod.m_reportingServices.forEach(endpoint ->
                    endpoint.push(event));
        });
    }

    public static void catchThrowable(final Object source, final Throwable exception) {
        final var encoded = stackTrace2Strings(exception.getStackTrace());

        if (INSTANCE.isPresent() && !Objects.isNull(source) && !(source instanceof ReportingEndpoint)) {
            // if the source isn't a reportable, make it one.
            final var verifiedSource = (source instanceof Reportable) ? (Reportable) source : Reportable.makeDummy(source.getClass());
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
