package com.gemsrobotics.frc2020.subsystems;

import com.gemsrobotics.frc2020.vision.TargetServer;
import com.gemsrobotics.lib.data.InterpolatingTreeMap;
import com.gemsrobotics.lib.math.interpolation.InterpolatingDouble;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.subsystems.drivetrain.FieldToVehicleEstimator;

import java.util.Objects;
import java.util.Optional;

public final class TargetState extends Subsystem {
	private static final double CACHE_DISTANCE_METERS = 8.0; // meters
	private static final int BUFFER_SIZE = 400;

	private static final RigidTransform
			VEHICLE_TO_TURRET = RigidTransform.identity(),
			TURRET_TO_CAMERA = RigidTransform.identity();

	private static TargetState INSTANCE;

	public static TargetState getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new TargetState();
		}

		return INSTANCE;
	}

	private final FieldToVehicleEstimator m_odometer;
	private final InterpolatingTreeMap<InterpolatingDouble, Rotation> m_turretHeading;
	private final PeriodicIO m_periodicIO;

	private TargetState() {
		m_odometer = Chassis.getInstance().getOdometer();
		m_turretHeading = new InterpolatingTreeMap<>(BUFFER_SIZE);

		m_periodicIO = new PeriodicIO();
	}

	public static class CachedTarget {
		public final double timestamp;
		public final Translation captureFieldToVehicle;

		public CachedTarget(final double timestamp, final Translation captureFieldToVehicle) {
			this.timestamp = timestamp;
			this.captureFieldToVehicle = captureFieldToVehicle;
		}
	}

	private static class PeriodicIO {
		private Rotation newTurretRotation = Rotation.identity();
		private Optional<TargetServer.TargetInfo> targetInfo = Optional.empty();
		private Optional<CachedTarget> fieldToTargetCached = Optional.empty();
	}

	@Override
	protected synchronized void readPeriodicInputs(final double timestamp) {
		m_periodicIO.newTurretRotation = Turret.getInstance().getRotation();
		m_periodicIO.targetInfo = TargetServer.getInstance().getTargetInfo();
	}

	@Override
	protected synchronized void onCreate(final double timestamp) {

	}

	@Override
	protected synchronized void onUpdate(final double now) {
		m_turretHeading.put(new InterpolatingDouble(now), m_periodicIO.newTurretRotation);

		if (m_periodicIO.targetInfo.isPresent()) {
			final var targetInfo = m_periodicIO.targetInfo.get();
			final var fieldToTarget = getFieldToCamera(now).transformBy(targetInfo.cameraToTarget);
			m_periodicIO.fieldToTargetCached = Optional.of(new CachedTarget(now, fieldToTarget.getTranslation()));
		} else if (m_periodicIO.fieldToTargetCached.isPresent()) {
			final var cachedLocation = m_periodicIO.fieldToTargetCached.get().captureFieldToVehicle.getTranslation();
			if (cachedLocation.difference(getFieldToVehicle(now).getTranslation()).norm() > CACHE_DISTANCE_METERS) {
				m_periodicIO.fieldToTargetCached = Optional.empty();
			}
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

	public synchronized Rotation getFieldToTurret(final double timestamp) {
		return getFieldToVehicle(timestamp).getRotation().rotateBy(getVehicleToTurret(timestamp));
	}

	private synchronized RigidTransform getFieldToCamera(final double timestamp) {
		return getFieldToVehicle(timestamp)
					   .transformBy(VEHICLE_TO_TURRET)
					   .transformBy(RigidTransform.fromRotation(getVehicleToTurret(timestamp)))
					   .transformBy(TURRET_TO_CAMERA);
	}

	public synchronized Optional<CachedTarget> getCachedFieldToTarget() {
		return m_periodicIO.fieldToTargetCached;
	}
}
