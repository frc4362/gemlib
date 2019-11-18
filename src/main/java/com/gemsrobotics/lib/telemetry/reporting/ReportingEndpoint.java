package com.gemsrobotics.lib.telemetry.reporting;

import com.gemsrobotics.lib.telemetry.Pod;

import java.util.LinkedList;
import java.util.Queue;

public abstract class ReportingEndpoint {
    public static final int REPORT_RATE_MS = 100;

    public static class Event {
        // ordered in importance
        public enum Kind {
            SUGGESTION, HYPOTHESIS, INQUIRY,
            INFO, ANNOUNCEMENT, WARNING, ERROR, HARDWARE_FAULT, SYSTEM
        }

        protected final String time;
        protected final Kind kind;
        protected final String source;
        protected final String text;
        protected final Object details;

        public Event(final String ti, final Kind k, final String s, final String t) {
            this(ti, k, s, t, null);
        }

        public Event(final String ti, final Kind k, final String s, final String t, final Object d) {
            kind = k;
            text = t;
            source = s;
            time = ti;
            details = d;
        }
    }

    protected final Queue<Event> m_eventsToReport;

    protected boolean m_halted;

    protected ReportingEndpoint() {
        m_eventsToReport = new LinkedList<>();
        m_halted = false;
    }

    public synchronized void halt() {
        synchronized (this) {
            m_halted = true;
        }
    }

    public synchronized boolean isHalted() {
        synchronized (this) {
            return m_halted;
        }
    }

    public final void push(final Event event) {
        if (filter(event)) {
            synchronized (this) {
                m_eventsToReport.add(event);
            }
        }
    }

    public final void flush() {
        if (!m_halted) {
            try {
                synchronized (this) {
                    if (m_eventsToReport.size() == 0) {
                        return;
                    }

                    doWrite(m_eventsToReport.toArray(Event[]::new));
                    m_eventsToReport.clear();
                }
            } catch (final Throwable throwable) {
                halt();
                Pod.catchThrowable(this, throwable);
            }
        }
    }

    protected boolean filter(final Event event) {
        return true;
    }

    protected abstract void doWrite(final Event[] events);
}
