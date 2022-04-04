package com.gemsrobotics.frc2022;

import com.gemsrobotics.lib.data.InterpolatingTreeMap;
import com.gemsrobotics.lib.math.interpolation.InterpolatingDouble;
import com.gemsrobotics.lib.math.se2.Rotation;

import java.util.AbstractMap;

public final class Constants {
	public static final double OPEN_LOOP_TURN_SENSITIVITY = 0.65;

	public static String SMARTDASHBOARD_HOOD_KEY = "Hood Reference Degrees";
	public static String SMARTDASHBOARD_SHOOTER_KEY = "Shooter Reference Velocity";

	private static class Entry extends AbstractMap.SimpleImmutableEntry<Double, ShooterConfiguration> {
		public Entry(final Double key, final ShooterConfiguration value) {
			super(key, value);
		}
	}

	public static final boolean DO_SHOOTER_TUNING = true;
	public static final boolean DO_SHOOTER_LOGGING = false;
	public static final boolean DO_EARLY_FLYWHEEL = true;
	public static final boolean DO_CARGO_REJECT = false;

	public static final boolean DO_RANGE_LOCK = false;
	private static final double SHOOTER_ALLOWED_MINIMUM_METERS = 1.7;
	private static final double SHOOTER_ALLOWED_MAXIMUM_METERS = 2.85;

	public static boolean isRangeOk(final double rangeMeters) {
		return rangeMeters > SHOOTER_ALLOWED_MINIMUM_METERS && rangeMeters < SHOOTER_ALLOWED_MAXIMUM_METERS;
	}

	// range -> (angle, velocity)
	private static final Entry[] SHOOTER_RANGE_ENTRIES = {
			new Entry(1.25, new ShooterConfiguration(Rotation.degrees(21.2), new InterpolatingDouble(9.15))),
			new Entry(1.4, new ShooterConfiguration(Rotation.degrees(21.2), new InterpolatingDouble(9.4))),
			new Entry(1.55, new ShooterConfiguration(Rotation.degrees(21.2), new InterpolatingDouble(9.6)))
	};

	private static final InterpolatingTreeMap<InterpolatingDouble, ShooterConfiguration> SHOOTER_RANGE_LERPER;
	static {
		SHOOTER_RANGE_LERPER = new InterpolatingTreeMap<>(SHOOTER_RANGE_ENTRIES.length);

		for (final var tuning : SHOOTER_RANGE_ENTRIES) {
			SHOOTER_RANGE_LERPER.put(new InterpolatingDouble(tuning.getKey()), tuning.getValue());
		}
	}

	public static ShooterConfiguration getShooterConfiguration(final double rangeMeters) {
		if (rangeMeters < SHOOTER_RANGE_ENTRIES[0].getKey()) {
			return SHOOTER_RANGE_ENTRIES[0].getValue();
		} else if (rangeMeters > SHOOTER_RANGE_ENTRIES[SHOOTER_RANGE_ENTRIES.length - 1].getKey()) {
			return SHOOTER_RANGE_ENTRIES[SHOOTER_RANGE_ENTRIES.length - 1].getValue();
		} else {
			return SHOOTER_RANGE_LERPER.getInterpolated(new InterpolatingDouble(rangeMeters));
		}
	}
}
