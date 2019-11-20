package com.gemsrobotics.frc2019.commands;

import com.gemsrobotics.frc2019.subsystems.drive.DifferentialDrive;
import com.gemsrobotics.frc2019.subsystems.drive.DifferentialDrive.Side;
import com.gemsrobotics.frc2019.util.motion.Rotation;
import com.gemsrobotics.frc2019.util.motion.Twist;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

import static java.lang.Math.abs;

@SuppressWarnings({"unused", "WeakerAccess"})
public final class ChassisLocalization extends Command {
	public static final double dt = 0.02;

	private double m_encoderPrevDistanceLeft;
	private double m_encoderPrevDistanceRight;

	private DifferentialDrive m_vehicle;

	public ChassisLocalization(final DifferentialDrive vehicle) {
		m_encoderPrevDistanceLeft = 0.0;
		m_encoderPrevDistanceRight = 0.0;

		m_vehicle = vehicle;
	}

	@Override
	public void initialize() {
		m_encoderPrevDistanceLeft = m_vehicle.getInchesPosition(Side.LEFT);
		m_encoderPrevDistanceRight = m_vehicle.getInchesPosition(Side.RIGHT);
	}

    private double deadband(final double d) {
	    return abs(d) > 0.01 ? d : 0.0;
    }

	@Override
	public void execute() {
		final double distanceLeft = m_vehicle.getInchesPosition(Side.LEFT),
				distanceRight = m_vehicle.getInchesPosition(Side.RIGHT);

		final Rotation angle = Rotation.fromDegrees(m_vehicle.getAHRS().getAngle());
		final Twist velocityMeasured = m_vehicle.getState().generateOdometry(
				deadband(distanceLeft - m_encoderPrevDistanceLeft),
				deadband(distanceRight - m_encoderPrevDistanceRight),
				angle);
		final Twist velocityPredicted = m_vehicle.getKinematics().forwardKinematics(
				m_vehicle.getInchesPerSecond(Side.LEFT) * dt,
				m_vehicle.getInchesPerSecond(Side.RIGHT) * dt);

		m_vehicle.getState().addObservations(
				Timer.getFPGATimestamp(),
				velocityMeasured,
				velocityPredicted);

		m_encoderPrevDistanceLeft = distanceLeft;
		m_encoderPrevDistanceRight = distanceRight;
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
