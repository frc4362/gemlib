package com.gemsrobotics.frc2020.subsystems;

import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.subsystems.Limelight;

import java.util.Objects;
import java.util.Optional;

import static java.lang.Math.tan;

public final class GoalServer extends Limelight {
	private static final double LENS_HEIGHT = 0.6096;
	private static final double GOAL_CENTER_HEIGHT = 2.49555;
	private static final Rotation LENS_PITCH_DEGREES = Rotation.degrees(10.0);

	private static GoalServer INSTANCE;

	public static GoalServer getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new GoalServer();
		}

		return INSTANCE;
	}

	@Override
	protected void onCreate(final double timestamp) {
		setCameraMode(CameraMode.COMPUTER_VISION);
		setLEDMode(LEDMode.OFF);
	}

	@Override
	protected void onUpdate(final double timestamp) {

	}

	@Override
	protected void onStop(final double timestamp) {
		//stop
	}

	public synchronized Optional<GoalInfo> getGoalInfo() {
		if (m_periodicIO.targetPresent) {
			final Rotation a2 = m_periodicIO.offsetVertical.sum(Rotation.radians(FOV_VERTICAL.getRadians() / 2.0));
			final double distanceMeters = (GOAL_CENTER_HEIGHT - LENS_HEIGHT) / tan(LENS_PITCH_DEGREES.sum(a2).getRadians());
			return Optional.of(new GoalInfo(m_periodicIO.timestamp, distanceMeters, Translation.identity()));
		} else {
			return Optional.empty();
		}
	}
}
