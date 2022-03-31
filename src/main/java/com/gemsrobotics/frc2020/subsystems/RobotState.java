package com.gemsrobotics.frc2020.subsystems;

import com.gemsrobotics.frc2020.Constants;
import com.gemsrobotics.frc2020.Target;
import com.gemsrobotics.lib.data.InterpolatingTreeMap;
import com.gemsrobotics.lib.math.interpolation.InterpolatingDouble;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.subsystems.drivetrain.FieldToVehicleEstimator;
import com.gemsrobotics.lib.utils.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Objects;
import java.util.Optional;

public final class RobotState extends Subsystem {
	private static final int BUFFER_SIZE = 400;
	private static final int MANY_METERS = 100;
	private static final Rotation INNER_SHOT_ALLOWED_DEFLECTION = Rotation.degrees(21.46);

	private static final RigidTransform
			VEHICLE_TO_TURRET = RigidTransform.fromTranslation(new Translation(Units.inches2Meters(-7.5), 0)),
			TURRET_TO_CAMERA = RigidTransform.fromTranslation(new Translation(Units.inches2Meters(9.04), 0));

	private static RobotState INSTANCE;

	public static RobotState getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new RobotState();
		}

		return INSTANCE;
	}

	private final FieldToVehicleEstimator m_odometer;
	private final InterpolatingTreeMap<InterpolatingDouble, Rotation> m_turretHeading;
	private final PeriodicIO m_periodicIO;

	private RobotState() {
		m_odometer = Chassis.getInstance().getOdometer();
		m_turretHeading = new InterpolatingTreeMap<>(BUFFER_SIZE);

		m_periodicIO = new PeriodicIO();
	}

	public class CachedTarget implements Target {
		private final Timer m_timer;
		private final Translation m_fieldToOuterGoal;
		private final double m_captureDriveDistance;

		public CachedTarget(final Translation fieldToTarget) {
			m_timer = new Timer();
			m_timer.start();

			m_fieldToOuterGoal = fieldToTarget;

			m_captureDriveDistance = m_odometer.getDistanceDriven();
		}

		public double getAge() {
			return m_timer.get();
		}

		public double getDriveDistanceSinceCapture() {
			return m_odometer.getDistanceDriven() - m_captureDriveDistance;
		}

		public Translation getVehicleGoal() {
			// L A T E N C Y
			final var turretPose = RobotState.this.getLatestFieldToVehicle()
				   .transformBy(VEHICLE_TO_TURRET)
				   .transformBy(RigidTransform.fromRotation(m_turretHeading.lastEntry().getValue()));
			return m_fieldToOuterGoal.difference(turretPose.getTranslation());
		}

		public Optional<Translation> getVehicleToInnerGoal() {
			final var innerGoal = getVehicleGoal().translateBy(Constants.OUTER_TO_INNER);
			final var innerA = innerGoal.translateBy(Translation.fromPolar(INNER_SHOT_ALLOWED_DEFLECTION, MANY_METERS).inverse());
			final var innerC = innerGoal.translateBy(Translation.fromPolar(INNER_SHOT_ALLOWED_DEFLECTION.inverse(), MANY_METERS).inverse());
			// get latest field to turret
			final var turretPose = RobotState.this.getLatestFieldToVehicle()
			   .transformBy(VEHICLE_TO_TURRET)
			   .transformBy(RigidTransform.fromRotation(m_turretHeading.lastEntry().getValue()));

			if (turretPose.getTranslation().isWithinAngle(innerA, innerGoal, innerC)) {
				return Optional.of(innerGoal);
			} else {
				return Optional.empty();
			}
		}

		public Translation getOptimalGoal() {
			return Constants.USE_INNER_ADJUSTMENT ? getVehicleToInnerGoal().orElseGet(this::getVehicleGoal) : getVehicleGoal();
		}
	}

	private static class PeriodicIO {
		public boolean targetServerAlive = false;
		public Rotation newTurretRotation = Rotation.identity();
		public Optional<TargetServer.TargetInfo> newTargetInfo = Optional.empty();
		public Optional<Target> fieldToTargetCached = Optional.empty();
	}

	@Override
	protected synchronized void readPeriodicInputs(final double timestamp) {
		m_periodicIO.newTurretRotation = ArmabotTurret240.getInstance().getRotation();
		m_periodicIO.targetServerAlive = TargetServer.getInstance().isAlive();
		m_periodicIO.newTargetInfo = TargetServer.getInstance().getTargetInfo();
	}

	@Override
	protected synchronized void onStart(final double timestamp) {
	}

	@Override
	protected synchronized void onUpdate(final double now) {
		m_turretHeading.put(new InterpolatingDouble(now), m_periodicIO.newTurretRotation);

		if (m_periodicIO.targetServerAlive && m_periodicIO.newTargetInfo.isPresent()) {
			final var newTarget = m_periodicIO.newTargetInfo.get();
			// this is where latency compensation happens
			SmartDashboard.putString("Camera to Target", newTarget.getCameraToTarget().toString());
			final var fieldToTarget = getFieldToCamera(newTarget.getTimestamp()).transformBy(newTarget.getCameraToTarget());
			m_periodicIO.fieldToTargetCached = Optional.of(new CachedTarget(fieldToTarget.getTranslation()));
		}
	}

	@Override
	protected synchronized void onStop(final double timestamp) {
	}

	@Override
	public void setSafeState() {
		// saf ety
	}

	public synchronized RigidTransform getFieldToVehicle(final double timestamp) {
		return m_odometer.getFieldToVehicle(timestamp);
	}

	public synchronized RigidTransform getLatestFieldToVehicle() {
		return m_odometer.getLatestFieldToVehicleValue();
	}

	public synchronized Rotation getVehicleToTurret(final double timestamp) {
		return m_turretHeading.getInterpolated(new InterpolatingDouble(timestamp));
	}

	public synchronized RigidTransform getFieldToTurret(final double timestamp) {
		return getFieldToVehicle(timestamp).transformBy(VEHICLE_TO_TURRET).transformBy(RigidTransform.fromRotation(getVehicleToTurret(timestamp)));
	}

	public synchronized RigidTransform getFieldToCamera(final double timestamp) {
		return getFieldToTurret(timestamp).transformBy(TURRET_TO_CAMERA);
	}

	public synchronized Optional<Target> getCachedFieldToTarget() {
		return m_periodicIO.fieldToTargetCached;
	}
}
