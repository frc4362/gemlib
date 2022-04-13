package com.gemsrobotics.frc2022.commands;

import com.gemsrobotics.frc2022.subsystems.Uptake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;

public final class ExhaustBallsCommand implements Command {
	private final Uptake m_uptake;
	private final Timer m_runtime;

	public ExhaustBallsCommand() {
		m_uptake = Uptake.getInstance();
		m_runtime = new Timer();
	}

	@Override
	public void initialize() {
		m_runtime.reset();
		m_runtime.start();
	}

	@Override
	public void execute() {
		if (m_runtime.hasElapsed(1.0)) {
			m_uptake.setWantedState(Uptake.State.UNLOADING);
		} else {
			m_uptake.setWantedState(Uptake.State.EXHAUSTING);
		}
	}

	@Override
	public boolean isFinished() {
		return m_runtime.hasElapsed(2.0);
	}

	@Override
	public void end(final boolean interrupted) {
		m_uptake.setWantedState(Uptake.State.NEUTRAL);
		m_runtime.stop();
	}

	@Override
	public Set<Subsystem> getRequirements() {
		return Set.of();
	}
}
