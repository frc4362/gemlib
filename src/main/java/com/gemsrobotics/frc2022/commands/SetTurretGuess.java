package com.gemsrobotics.frc2022.commands;

import com.gemsrobotics.frc2022.subsystems.Superstructure;
import com.gemsrobotics.lib.math.se2.Rotation;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetTurretGuess extends InstantCommand {
	private final Rotation m_guess;

	public SetTurretGuess(final Rotation guess) {
		m_guess = guess;
	}

	@Override
	public void initialize() {
		Superstructure.getInstance().setTurretGuess(m_guess);
	}
}
