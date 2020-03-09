package com.gemsrobotics.frc2020.commands;

import com.gemsrobotics.frc2020.subsystems.Superstructure;
import edu.wpi.first.wpilibj.command.Command;

public final class WaitForStateCommand extends Command {
	private final Superstructure m_superstructure;
	private final Superstructure.SystemState m_state;

	public WaitForStateCommand(final Superstructure superstructure, final Superstructure.SystemState state) {
		m_superstructure = superstructure;
		m_state = state;
	}

	@Override
	public boolean isFinished() {
		return m_superstructure.getSystemState() == m_state;
	}
}
