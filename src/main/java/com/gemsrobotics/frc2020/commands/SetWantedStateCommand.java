package com.gemsrobotics.frc2020.commands;

import com.gemsrobotics.frc2020.subsystems.Superstructure;
import edu.wpi.first.wpilibj.command.InstantCommand;

public final class SetWantedStateCommand extends InstantCommand {
	private final Superstructure m_superstructure;
	private final Superstructure.WantedState m_wantedState;

	public SetWantedStateCommand(final Superstructure superstructure, final Superstructure.WantedState wantedState) {
		m_superstructure = superstructure;
		m_wantedState = wantedState;
	}

	@Override
	public void initialize() {
		m_superstructure.setWantedState(m_wantedState);
	}
}
