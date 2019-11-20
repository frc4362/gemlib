package com.gemsrobotics.frc2019.subsystems.drive;

import com.gemsrobotics.frc2019.util.math.InterpolateDouble;
import com.gemsrobotics.frc2019.util.math.InterpolatingTreeMap;
import com.gemsrobotics.frc2019.util.motion.Pose;
import com.gemsrobotics.frc2019.util.motion.Rotation;
import com.gemsrobotics.frc2019.util.motion.Twist;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Map;

@SuppressWarnings({"unused", "WeakerAccess"})
public final class RobotState {
	private static final int BUFFER_SIZE = 100;

	private final DifferentialDrive m_chassis;
	private InterpolatingTreeMap<InterpolateDouble, Pose> m_fieldToVehicle;
	private Twist m_vehicleVelocityPredicted;
	private double m_distanceDriven;

	public RobotState(final DifferentialDrive chassis) {
		m_chassis = chassis;
		reset(0, Pose.identity());
	}

	public void reset(final double startTime, final Pose initialFieldToVehicle) {
		m_fieldToVehicle = new InterpolatingTreeMap<>(BUFFER_SIZE);
		m_fieldToVehicle.put(new InterpolateDouble(startTime), initialFieldToVehicle);
		m_vehicleVelocityPredicted = Twist.identity();
		resetDistance();
	}

	public void resetDistance() {
		m_distanceDriven = 0.0;
	}

	public Map.Entry<InterpolateDouble, Pose> getLatestFieldToVehicle() {
		return m_fieldToVehicle.lastEntry();
	}

	private void addFieldToVehicleObservation(final double timestamp, final Pose frame) {
		m_fieldToVehicle.put(new InterpolateDouble(timestamp), frame);
	}

	public void addObservations(
			final double timestamp,
			final Twist velocityMeasured,
			final Twist velocityPredicted
	) {
		final var integratedPose = m_chassis.getKinematics().integrateForwardKinematics(
				getLatestFieldToVehicle().getValue(),
				velocityMeasured);

		addFieldToVehicleObservation(timestamp, integratedPose);
		m_vehicleVelocityPredicted = velocityPredicted;
	}

	public Twist generateOdometry(
			final double encoderDeltaLeft,
			final double encoderDeltaRight,
			final Rotation heading
	) {
		final Pose measurementLast = getLatestFieldToVehicle().getValue();
		final Twist delta = m_chassis.getKinematics().forwardKinematics(
				measurementLast.getRotation(),
				encoderDeltaLeft,
				encoderDeltaRight,
				heading);

		m_distanceDriven += delta.dx;

		return delta;
	}

	public double getDistanceDriven() {
		return m_distanceDriven;
	}

	public Twist getPredictedVelocity() {
		return m_vehicleVelocityPredicted;
	}

	private static final String REPR_STRING = "RobotState[latest: %s, predicted: %s]";

	@Override
	public String toString() {
		return String.format(
				REPR_STRING,
				getLatestFieldToVehicle().getValue().toString(),
				getPredictedVelocity().toString());
	}

	private static class RobotStateLogger extends Command {
		private final RobotState m_state;

		public RobotStateLogger(final RobotState state) {
			m_state = state;
		}

		@Override
		public void execute() {
			final Pose pose = m_state.getLatestFieldToVehicle().getValue();
			SmartDashboard.putNumber("X Position (Inches)", pose.getTranslation().x());
			SmartDashboard.putNumber("Y Position (Inches)", pose.getTranslation().y());
			SmartDashboard.putNumber("Heading", pose.getRotation().getDegrees());
			SmartDashboard.putNumber("Distance Driven", m_state.getDistanceDriven());
		}

		@Override
		public boolean isFinished() {
			return false;
		}
	}

	public RobotStateLogger makeLogger() {
		return new RobotStateLogger(this);
	}
}
