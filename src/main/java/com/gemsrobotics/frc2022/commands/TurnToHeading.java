package com.gemsrobotics.frc2022.commands;

import com.gemsrobotics.frc2022.subsystems.Chassis;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.subsystems.drivetrain.WheelState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;

public class TurnToHeading implements Command {
	private static final double kP = 0.005;
	private static final double kFF = 0.03;
	private final Chassis m_chassis;
	private final Rotation m_goal;
	private Rotation m_error;

	public TurnToHeading(final Rotation goal) {
		m_chassis = Chassis.getInstance();
		m_goal = goal;
	}

	@Override
	public void execute() {
		m_error = m_goal.difference(m_chassis.getHeading());
		final var e = m_error.getDegrees();
		m_chassis.setVoltages(new WheelState(e * -kP - kFF * e, e * kP + kFF * e));
	}

	@Override
	public boolean isFinished() {
		return Math.abs(m_error.getDegrees()) < 2.0;
	}

	@Override
	public Set<Subsystem> getRequirements() {
		return Set.of();
	}

	@Override
	public void end(final boolean interrupted) {
		m_chassis.setDisabled();
	}
}
