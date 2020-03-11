package com.gemsrobotics.frc2020.subsystems;

import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.subsystems.Limelight;
import com.gemsrobotics.lib.utils.Units;

import java.util.*;

import static java.lang.Math.tan;

public final class TargetServer extends Limelight {
	private static final double LENS_HEIGHT = 0.64;
	// TODO add this back if we're back to targeting the center of the goal, rather than the top edge
	private static final double GOAL_CENTER_HEIGHT = 2.49555; //  - Units.inches2Meters(9.5);

	private static TargetServer INSTANCE;

	public static TargetServer getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new TargetServer();
		}

		return INSTANCE;
	}

	private TargetServer() {
		super(Resolution.LOW_DEF, Rotation.degrees(31.0), LENS_HEIGHT, GOAL_CENTER_HEIGHT);
	}

	public static class GoalState {
		private final double m_timestamp;
		private final RigidTransform m_cameraToTarget;

		public GoalState(final double timestamp, final RigidTransform cameraToTarget) {
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
}
