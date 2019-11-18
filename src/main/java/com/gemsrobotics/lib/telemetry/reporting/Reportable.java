package com.gemsrobotics.lib.telemetry.reporting;

import com.gemsrobotics.lib.telemetry.Pod;

public interface Reportable {
	default String getName() {
	    return getClass().getSimpleName();
    }

	default void report(final String message) {
		report(ReportingEndpoint.Event.Kind.INFO, message);
	}

	default void report(final ReportingEndpoint.Event.Kind kind, final String message) {
		report(kind, message, null);
	}

	default void report(final ReportingEndpoint.Event.Kind kind, final String message, final Object details) {
		Pod.log(kind, this, message, details);
	}

	static Reportable makeDummy(final String name) {
		return new Reportable() {
            @Override
            public String getName() {
                return name;
            }
        };
	}

	static Reportable makeDummy(final Class<?> type) {
	    return makeDummy(type.getSimpleName());
    }
}
