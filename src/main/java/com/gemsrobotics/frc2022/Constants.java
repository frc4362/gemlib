package com.gemsrobotics.frc2022;

import com.gemsrobotics.lib.data.InterpolatingTreeMap;
import com.gemsrobotics.lib.math.interpolation.InterpolatingDouble;
import com.gemsrobotics.lib.math.se2.Rotation;

import java.util.AbstractMap;

public final class Constants {
	public static final double OPEN_LOOP_TURN_SENSITIVITY = 1;// 0.65;
	public static final double CLIMB_EXTEND_THRESHOLD_DEGREES = 41.5;

	public static String SMARTDASHBOARD_HOOD_KEY = "Hood Reference Degrees";
	public static String SMARTDASHBOARD_SHOOTER_KEY = "Shooter Reference Velocity";

	private static class Entry extends AbstractMap.SimpleImmutableEntry<Double, ShooterConfiguration> {
		public Entry(final Double key, final ShooterConfiguration value) {
			super(key, value);
		}
	}

	public static final boolean DO_SHOOTER_TUNING = false;
	public static final boolean DO_SHOOTER_LOGGING = true;
	public static final boolean DO_EARLY_FLYWHEEL = true;
	public static final boolean DO_CARGO_REJECT = true;

	public static final boolean DO_RANGE_LOCK = true;
	private static final double SHOOTER_ALLOWED_MINIMUM_METERS = 1.25;
	private static final double SHOOTER_ALLOWED_MAXIMUM_METERS = 10.0;

	public static final Rotation SAFE_AUTO_HOOD_ANGLE = Rotation.degrees(33.0);
	public static final ShooterConfiguration FENDER_SHOT_CONFIGURATION = new ShooterConfiguration(Rotation.degrees(46), new InterpolatingDouble(5.0));

	public static boolean isRangeOk(final double rangeMeters) {
		return rangeMeters > SHOOTER_ALLOWED_MINIMUM_METERS && rangeMeters < SHOOTER_ALLOWED_MAXIMUM_METERS;
	}

	// range -> (angle, velocity)
	private static final Entry[] SHOOTER_RANGE_ENTRIES = {
			new Entry(1.25, new ShooterConfiguration(Rotation.degrees(22), new InterpolatingDouble(8.5))),
			new Entry(1.59, new ShooterConfiguration(Rotation.degrees(27), new InterpolatingDouble(8.25))),
			new Entry(1.7, new ShooterConfiguration(Rotation.degrees(28.5), new InterpolatingDouble(8.5))),
			new Entry(1.95, new ShooterConfiguration(Rotation.degrees(31), new InterpolatingDouble(9.0))),
			new Entry(2.2, new ShooterConfiguration(Rotation.degrees(33), new InterpolatingDouble(9.17))),
			new Entry(2.48, new ShooterConfiguration(Rotation.degrees(33), new InterpolatingDouble(9.65))),
			new Entry(2.8, new ShooterConfiguration(Rotation.degrees(34), new InterpolatingDouble(9.8))),
			new Entry(3.16, new ShooterConfiguration(Rotation.degrees(35), new InterpolatingDouble(10.25))),
			new Entry(3.47, new ShooterConfiguration(Rotation.degrees(35), new InterpolatingDouble(11.5))),
			new Entry(3.9, new ShooterConfiguration(Rotation.degrees(46), new InterpolatingDouble(10.9))),
			new Entry(4.47, new ShooterConfiguration(Rotation.degrees(46), new InterpolatingDouble(12.0))),
			new Entry(6.2, new ShooterConfiguration(Rotation.degrees(46), new InterpolatingDouble(13.9))),
			new Entry(6.5, new ShooterConfiguration(Rotation.degrees(46), new InterpolatingDouble(14.2))),
			new Entry(7.9, new ShooterConfiguration(Rotation.degrees(46), new InterpolatingDouble(15.3))),
			new Entry(10.0, new ShooterConfiguration(Rotation.degrees(46), new InterpolatingDouble(16.5))),
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
