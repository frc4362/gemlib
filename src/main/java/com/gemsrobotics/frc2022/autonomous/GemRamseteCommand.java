package com.gemsrobotics.frc2022.autonomous;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import com.gemsrobotics.frc2022.subsystems.Chassis;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.subsystems.drivetrain.WheelState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

public class GemRamseteCommand implements Command {
	private final Timer m_timer = new Timer();
	private final Chassis m_chassis;
	private final Trajectory m_trajectory;
	private final Supplier<Pose2d> m_pose;
	private final RamseteController m_follower;
	private final SimpleMotorFeedforward m_feedforward;
	private final DifferentialDriveKinematics m_kinematics;
	private final Supplier<DifferentialDriveWheelSpeeds> m_speeds;
	private final PIDController m_leftController;
	private final PIDController m_rightController;
	private final BiConsumer<Double, Double> m_output;
	private DifferentialDriveWheelSpeeds m_prevSpeeds;
	private double m_prevTime;

	public GemRamseteCommand(final Trajectory trajectory) {
		m_chassis = Chassis.getInstance();
		m_trajectory = requireNonNullParam(trajectory, "trajectory", "RamseteCommand");
		m_pose = () -> {
			final var rt = m_chassis.getOdometer().getLatestFieldToVehicleValue();
			final var t = rt.getTranslation();
			return new Pose2d(new Translation2d(t.x(), t.y()), Rotation2d.fromDegrees(rt.getRotation().getDegrees()));
		};

		m_follower = new RamseteController(2.0, 0.7);
		m_feedforward = new SimpleMotorFeedforward(Chassis.kS, Chassis.kV, Chassis.linearKa);
		m_kinematics = new DifferentialDriveKinematics(Chassis.trackWidth);

		m_speeds = () -> {
			final var speeds = m_chassis.getWheelProperty(MotorController::getVelocityLinearMetersPerSecond);
			return new DifferentialDriveWheelSpeeds(speeds.left, speeds.right);
		};

		m_leftController = m_chassis.makePIDController();
		m_rightController = m_chassis.makePIDController();
		m_output = (l, r) -> m_chassis.setVoltages(new WheelState(l, r));
	}

	@Override
	public void initialize() {
		m_prevTime = -1;
		var initialState = m_trajectory.sample(0);
		m_prevSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(initialState.velocityMetersPerSecond, 0, initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
		m_timer.reset();
		m_timer.start();
		m_leftController.reset();
		m_rightController.reset();
	}

	@Override
	public void execute() {
		double curTime = m_timer.get();
		double dt = curTime - m_prevTime;

		if (m_prevTime < 0) {
			m_output.accept(0.0, 0.0);
			m_prevTime = curTime;
			return;
		}

		var targetWheelSpeeds = m_kinematics.toWheelSpeeds(m_follower.calculate(m_pose.get(), m_trajectory.sample(curTime)));

		var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
		var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

		double leftOutput;
		double rightOutput;

		double leftFeedforward = m_feedforward.calculate(leftSpeedSetpoint, (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);
		double rightFeedforward = m_feedforward.calculate(rightSpeedSetpoint, (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);

		leftOutput = leftFeedforward + m_leftController.calculate(m_speeds.get().leftMetersPerSecond, leftSpeedSetpoint);
		rightOutput = rightFeedforward + m_rightController.calculate(m_speeds.get().rightMetersPerSecond, rightSpeedSetpoint);

		m_output.accept(leftOutput, rightOutput);
		m_prevSpeeds = targetWheelSpeeds;
		m_prevTime = curTime;
	}

	@Override
	public void end(final boolean interrupted) {
		m_timer.stop();
		m_output.accept(0.0, 0.0);
	}

	@Override
	public boolean isFinished() {
		return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
	}

	@Override
	public Set<Subsystem> getRequirements() {
		return Set.of();
	}
}