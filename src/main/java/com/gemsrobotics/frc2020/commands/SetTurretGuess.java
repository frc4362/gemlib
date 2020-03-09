package com.gemsrobotics.frc2020.commands;

import com.gemsrobotics.frc2020.subsystems.Superstructure;
import com.gemsrobotics.lib.math.se2.Rotation;
import edu.wpi.first.wpilibj.command.InstantCommand;

public final class SetTurretGuess extends InstantCommand {
	private final Superstructure m_superstructure;
	private final Rotation m_guess;

	public SetTurretGuess(final Superstructure superstructure, final Rotation guess) {
		m_superstructure = superstructure;
		m_guess = guess;
	}

	@Override
	public void initialize() {
		m_superstructure.setTurretGuess(m_guess);
	}
}
