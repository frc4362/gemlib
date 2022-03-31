package com.gemsrobotics.frc2022.commands;

import com.gemsrobotics.frc2022.subsystems.Intake;
import com.gemsrobotics.frc2022.subsystems.Superstructure;
import com.gemsrobotics.frc2022.subsystems.Uptake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;

public class IntakeCommand implements Command {
	private final Uptake m_uptake;
	private final Superstructure m_superstructure;
	private final int m_endBallCount;
	private final Timer m_timer;
	private final double m_timeout;

	public IntakeCommand(final int endBallCount, final double timeout) {
		m_uptake = Uptake.getInstance();
		m_superstructure = Superstructure.getInstance();
		m_endBallCount = endBallCount;
		m_timeout = timeout;
		m_timer = new Timer();
	}

	@Override
	public void initialize() {
		m_timer.reset();
		m_timer.start();
	}

	@Override
	public void execute() {
		m_superstructure.setWantedState(Superstructure.WantedState.INTAKING);
	}

	@Override
	public boolean isFinished() {
		return m_uptake.getBallCount() == m_endBallCount || m_timer.hasElapsed(m_timeout);
	}

	@Override
	public Set<Subsystem> getRequirements() {
		return Set.of();
	}

	@Override
	public void end(final boolean interrupted) {
		m_superstructure.setWantedState(Superstructure.WantedState.IDLE);
		m_timer.stop();
	}
}
