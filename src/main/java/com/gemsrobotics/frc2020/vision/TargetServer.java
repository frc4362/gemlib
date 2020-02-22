package com.gemsrobotics.frc2020.vision;

import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.subsystems.Limelight;

import java.util.*;

import static java.lang.Math.tan;

public final class TargetServer extends Limelight {
	private static final double LENS_HEIGHT = 0.57;
	private static final double GOAL_CENTER_HEIGHT = 2.49555;
	private static final Rotation LENS_PITCH_DEGREES = Rotation.degrees(10.0);

	private static TargetServer INSTANCE;

	public static TargetServer getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new TargetServer();
		}

		return INSTANCE;
	}

	public static class TargetInfo {
		public final double timestamp;
		public final RigidTransform cameraToTarget;

		public TargetInfo(final double timestamp, final RigidTransform cameraToTarget) {
			this.timestamp = timestamp;
			this.cameraToTarget = cameraToTarget;
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
			final Rotation a2 = m_periodicIO.offsetVertical.sum(FOV_VERTICAL_HALF);
			final double distanceMeters = (GOAL_CENTER_HEIGHT - LENS_HEIGHT) / tan(LENS_PITCH_DEGREES.sum(a2).getRadians());
			final RigidTransform cameraToTarget = new RigidTransform(
					Translation.fromPolar(m_periodicIO.offsetHorizontal, distanceMeters),
					m_periodicIO.offsetHorizontal);

			return Optional.of(new TargetInfo(m_periodicIO.timestamp, cameraToTarget));
		} else {
			return Optional.empty();
		}
	}
}
