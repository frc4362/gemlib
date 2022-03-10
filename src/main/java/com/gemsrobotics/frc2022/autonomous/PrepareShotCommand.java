package com.gemsrobotics.frc2022.autonomous;

import com.gemsrobotics.frc2022.subsystems.Superstructure;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class PrepareShotCommand extends InstantCommand {
	private final boolean m_prepareShot;

	public PrepareShotCommand(final boolean doPrepareShot) {
		m_prepareShot = doPrepareShot;
	}

	@Override
	public void initialize() {
		Superstructure.getInstance().setPrepareShot(m_prepareShot);
	}
}
