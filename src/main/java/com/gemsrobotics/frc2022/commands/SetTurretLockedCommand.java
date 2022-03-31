package com.gemsrobotics.frc2022.commands;

import com.gemsrobotics.frc2022.subsystems.Superstructure;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetTurretLockedCommand extends InstantCommand {
	private final boolean m_locked;

	public SetTurretLockedCommand(final boolean locked) {
		m_locked = locked;
	}

	@Override
	public void initialize() {
		Superstructure.getInstance().setTurretLocked(m_locked);
	}
}
