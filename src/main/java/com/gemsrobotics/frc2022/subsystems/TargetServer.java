package com.gemsrobotics.frc2022.subsystems;

import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.subsystems.Limelight;
import com.gemsrobotics.lib.utils.Units;

import java.util.*;

import static java.lang.Math.tan;

public final class TargetServer extends Limelight {
	private static final double LENS_HEIGHT_METERS = Units.inches2Meters(43);
	private static final double GOAL_CENTER_HEIGHT_METERS = 2.581275;
	private static final Rotation LENS_PITCH = Rotation.degrees(30.0);

	private static TargetServer INSTANCE;

	public static TargetServer getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new TargetServer();
		}

		return INSTANCE;
	}

	public static class TargetInfo {
		private final double m_timestamp;
		private final RigidTransform m_cameraToTarget;

		public TargetInfo(final double timestamp, final RigidTransform cameraToTarget) {
			m_timestamp = timestamp;
			m_cameraToTarget = cameraToTarget;
		}

		public double getTimestamp() {
			return m_timestamp;
		}

		public RigidTransform getCameraToTarget() {
			return m_cameraToTarget;
		}
	}

	@Override
	protected void onStart(final double timestamp) {
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

	public synchronized Optional<TargetInfo> getTargetInfo() {
		if (m_periodicIO.targetPresent) {
			final Rotation a2 = m_periodicIO.offsetVertical;
			final double distanceMeters = ((GOAL_CENTER_HEIGHT_METERS - LENS_HEIGHT_METERS) / tan(LENS_PITCH.sum(a2).getRadians()));
			// TODO
//			final double correctedDistanceMeters = (distanceMeters / 1.2) + 0.2968395564; // thanks regression
			final RigidTransform cameraToTarget = RigidTransform.fromTranslation(Translation.fromPolar(m_periodicIO.offsetHorizontal, distanceMeters));

			return Optional.of(new TargetInfo(m_periodicIO.timestamp, cameraToTarget));
		} else {
			return Optional.empty();
		}
	}
}