package com.gemsrobotics.frc2020.commands;

import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.subsystems.drivetrain.ChassisState;
import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDrive;
import com.gemsrobotics.lib.timing.ElapsedTimer;
import edu.wpi.first.wpilibj.command.Command;

public class DriveStraightCommand extends Command {
	public static final double kP = -0.0125;
	private final DifferentialDrive<?> m_chassis;
	private final double m_power, m_distance;

	private double m_startingPosition;

	private Rotation m_startingHeading;

	public DriveStraightCommand(final DifferentialDrive<?> chassis, final double power, final double distance) {
		m_chassis = chassis;
		m_power = power;
		m_distance = distance;
	}

	@Override
	public void initialize() {
		final var d = m_chassis.getWheelProperty(MotorController::getPositionMeters).left;
		m_startingPosition = d;
		m_startingHeading = m_chassis.getHeading();
	}

	@Override
	public void execute() {
		m_chassis.setOpenLoop(new ChassisState(m_power, m_chassis.getHeading().difference(m_startingHeading).getDegrees() * kP));
	}

	@Override
	public boolean isFinished() {
		final var d = m_chassis.getWheelProperty(MotorController::getPositionMeters).left;
		return (d - m_startingPosition) > m_distance;
	}

	@Override
	public void end() {
		m_chassis.setDisabled();
	}
}
