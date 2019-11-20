package com.gemsrobotics.frc2019.commands;

import com.gemsrobotics.frc2019.subsystems.lift.Lift;
import edu.wpi.first.wpilibj.command.Command;

public class LiftUnstucker extends Command {
	private final Lift m_lift;

	public LiftUnstucker(final Lift lift) {
		m_lift = lift;
	}

	@Override
	public void execute() {
		if (m_lift.isAtSetpoint()) {
			m_lift.pause();
		} else {
			m_lift.adjustPosition(0.0);
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
