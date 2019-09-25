package com.gemsrobotics.lib.telemetry.reporting;

import com.gemsrobotics.lib.telemetry.Pod;

public interface Reportable {
	default String getName() {
	    return getClass().getSimpleName();
    }

	default void report(final String str) {
		report(Reporter.Event.Kind.INFO, str);
	}

	default void report(final Reporter.Event.Kind kind, final String str) {
		report(kind, str, null);
	}

	default void report(final Reporter.Event.Kind kind, final String str, final Object details) {
		Pod.log(kind, this, str, details);
	}

	static Reportable makeDummy(final String name) {
		return new Reportable() {
            @Override
            public String getName() {
                return name;
            }
        };
	}
}
