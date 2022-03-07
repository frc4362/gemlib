package com.gemsrobotics.frc2022;

import com.gemsrobotics.frc2022.subsystems.Chassis;
import com.gemsrobotics.frc2022.subsystems.TargetServer;
import com.gemsrobotics.frc2022.subsystems.GreyTTurret;
import com.gemsrobotics.lib.data.InterpolatingTreeMap;
import com.gemsrobotics.lib.math.interpolation.InterpolatingDouble;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.subsystems.drivetrain.FieldToVehicleEstimator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Objects;
import java.util.Optional;

public final class FieldState extends Subsystem {
	private static final int BUFFER_SIZE = 400;
	private static final int MANY_METERS = 100;

	private static final RigidTransform
			VEHICLE_TO_TURRET = RigidTransform.fromTranslation(new Translation(-0.205, 0)), // meters
			TURRET_TO_CAMERA = RigidTransform.fromTranslation(new Translation(0.22, 0));

	private static FieldState INSTANCE;

	public static FieldState getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new FieldState();
		}

		return INSTANCE;
	}

	private final FieldToVehicleEstimator m_odometer;
	private final InterpolatingTreeMap<InterpolatingDouble, Rotation> m_turretHeading;
	private final PeriodicIO m_periodicIO;

	private FieldState() {
		m_odometer = Chassis.getInstance().getOdometer();
		m_turretHeading = new InterpolatingTreeMap<>(BUFFER_SIZE);

		m_periodicIO = new PeriodicIO();
	}

	public class CachedTarget implements ShotParameters {
		private final Timer m_timer;
		private final Translation m_fieldToGoal;
		private final double m_captureDriveDistance;

		public CachedTarget(final Translation fieldToTarget) {
			m_timer = new Timer();
			m_timer.start();

			m_fieldToGoal = fieldToTarget;

			m_captureDriveDistance = m_odometer.getDistanceDriven();
		}

		@Override
		public double getAgeSeconds() {
			return m_timer.get();
		}

		public double getDriveDistanceSinceCapture() {
			return m_odometer.getDistanceDriven() - m_captureDriveDistance;
		}

		@Override
		public Translation getCurrentTurretToGoal() {
			// L A T E N C Y
			final var turretPose = FieldState.this.getLatestFieldToVehicle()
				   .transformBy(VEHICLE_TO_TURRET)
				   .transformBy(RigidTransform.fromRotation(m_turretHeading.lastEntry().getValue()));
			return m_fieldToGoal.difference(turretPose.getTranslation());
		}
	}

	private static class PeriodicIO {
		public boolean isTargetServerAlive = false;
		public Rotation newTurretRotation = Rotation.identity();
		public Optional<TargetServer.TargetInfo> newTargetInfo = Optional.empty();
		public Optional<ShotParameters> fieldToTargetCached = Optional.empty();
	}

	@Override
	protected void readPeriodicInputs(final double timestamp) {
		m_periodicIO.newTurretRotation = GreyTTurret.getInstance().getRotation();
		m_periodicIO.isTargetServerAlive = TargetServer.getInstance().isAlive();
		m_periodicIO.newTargetInfo = TargetServer.getInstance().getTargetInfo();
	}

	@Override
	protected void onStart(final double timestamp) {
	}

	@Override
	protected void onUpdate(final double now) {
		m_turretHeading.put(new InterpolatingDouble(now), m_periodicIO.newTurretRotation);

		SmartDashboard.putBoolean("Target Server/Alive", m_periodicIO.isTargetServerAlive);
		SmartDashboard.putBoolean("Target Server/New Data Present", m_periodicIO.newTargetInfo.isPresent());

		if (m_periodicIO.isTargetServerAlive && m_periodicIO.newTargetInfo.isPresent()) {
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

	public RigidTransform getFieldToVehicle(final double timestamp) {
		return m_odometer.getFieldToVehicle(timestamp);
	}

	public RigidTransform getLatestFieldToVehicle() {
		return m_odometer.getLatestFieldToVehicleValue();
	}

	public Rotation getVehicleToTurret(final double timestamp) {
		return m_turretHeading.getInterpolated(new InterpolatingDouble(timestamp));
	}

	public RigidTransform getFieldToTurret(final double timestamp) {
		return getFieldToVehicle(timestamp).transformBy(VEHICLE_TO_TURRET).transformBy(RigidTransform.fromRotation(getVehicleToTurret(timestamp)));
	}

	public RigidTransform getFieldToCamera(final double timestamp) {
		return getFieldToTurret(timestamp).transformBy(TURRET_TO_CAMERA);
	}

	public Optional<ShotParameters> getCachedFieldToTarget() {
		return m_periodicIO.fieldToTargetCached;
	}
}
