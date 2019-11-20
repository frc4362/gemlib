package com.gemsrobotics.frc2019.commands;

import com.gemsrobotics.frc2019.subsystems.lift.Lift;
import edu.wpi.first.wpilibj.command.Command;

@SuppressWarnings("WeakerAccess")
public class LiftMovement extends Command {
	private int m_runs;

	private final Lift m_lift;
	private final Lift.Position m_position;

	public LiftMovement(
			final Lift lift,
			final Lift.Position position
	) {
		m_lift = lift;
		m_position = position;
	}

	@Override
	public void initialize() {
		m_lift.setPosition(m_position);
		m_runs = 0;
	}

	@Override
	public void execute() {
		m_runs++;
	}

	@Override
	public boolean isFinished() {
		return m_runs > 1 && m_lift.isAtSetpoint();
	}
}
