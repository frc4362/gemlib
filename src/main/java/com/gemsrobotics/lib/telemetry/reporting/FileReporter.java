package com.gemsrobotics.lib.telemetry.reporting;

import com.gemsrobotics.lib.telemetry.Pod;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.*;

public final class FileReporter extends ReportingEndpoint {
    private static final String OUTPUT_DIR = "/home/lvuser/eventlogs/";

    private static FileReporter INSTANCE;

    public static FileReporter getInstance() {
        if (Objects.isNull(INSTANCE)) {
            INSTANCE = new FileReporter();
        }

        return INSTANCE;
    }

    private static class Session {
        private final String runtime;
        private final List<Event> events;

        private Session(final String timestamp) {
            runtime = timestamp;
            events = new ArrayList<>();
        }
    }

    private final Gson m_eventEntrySerializer;
    private final Session m_eventSession;
    private final Optional<FileChannel> m_channel;

    private FileReporter() {
        final var runtime = Pod.getLaunchTime();

        m_eventEntrySerializer = new GsonBuilder().setPrettyPrinting().create();
        m_eventSession = new Session(runtime);

        final var outputPath = Paths.get(OUTPUT_DIR + runtime + ".json");
        Optional<FileChannel> channel;

        try {
            channel = Optional.of(FileChannel.open(outputPath, StandardOpenOption.CREATE, StandardOpenOption.APPEND));
        } catch (final IOException ioException) {
            ioException.printStackTrace();
            channel = Optional.empty();
        }

        m_channel = channel;
    }

    @Override
    protected boolean filter(final Event event) {
        return event.kind.ordinal() >= Event.Kind.WARNING.ordinal();
    }

    @Override
    protected void doWrite(final Event[] events) {
        if (m_channel.isEmpty()) {
            halt();
            return;
        }

        m_eventSession.events.addAll(Arrays.asList(events));

        final var serializedEvents = m_eventEntrySerializer.toJson(m_eventSession);
        final var bytes = ByteBuffer.wrap(serializedEvents.getBytes());

        try {
            m_channel.get().truncate(0).write(bytes);
        } catch (final Throwable exception) {
            exception.printStackTrace();
            halt();
        }
    }
}
