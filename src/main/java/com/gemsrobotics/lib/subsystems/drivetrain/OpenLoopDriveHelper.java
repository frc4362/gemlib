package com.gemsrobotics.lib.subsystems.drivetrain;


import static java.lang.Math.*;

@SuppressWarnings({"unused", "WeakerAccess"})
public class OpenLoopDriveHelper {
	public static class Config {
		// Lower this if quickturn is too aggressive
		public double quickTurnScalar = 1.0;
	}

	protected final transient OpenLoopDriveHelper.Config m_cfg;

	public OpenLoopDriveHelper(final OpenLoopDriveHelper.Config config) {
		m_cfg = config;
	}

	public WheelState drive(
			double throttle,
			double wheel,
			final boolean isQuickTurn,
			final boolean isHighGear
	) {
		final WheelState output;

		if (isQuickTurn) {
			output = new WheelState(throttle - wheel, throttle + wheel);
		} else {
			output = new WheelState(
					throttle + abs(throttle) * wheel * m_cfg.quickTurnScalar,
					throttle - abs(throttle) * wheel * m_cfg.quickTurnScalar);
		}

		final double maxMagnitude = max(1.0, max(abs(output.left), abs(output.right)));
		return output.map(p -> p / maxMagnitude);
	}
}
