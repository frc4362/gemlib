package com.gemsrobotics.lib.telemetry.reporting;

import com.google.gson.Gson;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.List;
import java.util.Objects;
import java.util.stream.Stream;

import static com.gemsrobotics.lib.telemetry.reporting.ReportingEndpoint.Event.Kind.*;

public final class ConsoleReporter extends ReportingEndpoint {
    private static ConsoleReporter INSTANCE;

    public static ConsoleReporter getInstance() {
        if (Objects.isNull(INSTANCE)) {
            INSTANCE = new ConsoleReporter();
        }

        return INSTANCE;
    }

    private static final Gson CONSOLE_SERIALIZER = new Gson();

    private ConsoleReporter() {
    }

    @Override
    public boolean filter(final Event event) {
        return List.of(INFO, WARNING, ERROR, HARDWARE_FAULT, ANNOUNCEMENT).contains(event.kind);
    }

    @Override
    public void doWrite(final Event[] events) {
        Stream.of(events).forEach(event -> {
            final var output = event2String(event);

            switch (event.kind) {
                case WARNING:
                    DriverStation.reportWarning(output, false);
                    break;
                case ERROR:
                    DriverStation.reportError(output, false);
                    break;
                default:
                    System.out.println(output);
                    break;
            }
        });
    }

    public static String event2String(final Event event) {
        final StringBuilder sb = new StringBuilder();
        sb.append('[');
        sb.append(event.kind.toString());
        sb.append("]: ");

        if (!Objects.isNull(event.source)) {
            sb.append('[');
            sb.append(event.source);
            sb.append("] ");
        }

        sb.append(event.text);

        if (!Objects.isNull(event.details)) {
            sb.append('\n');
            sb.append(" - ");
            sb.append(CONSOLE_SERIALIZER.toJson(event.details));
        }

        return sb.toString();
    }
}
