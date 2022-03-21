package com.gemsrobotics.frc2022.commands;

import com.gemsrobotics.frc2022.subsystems.Superstructure;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;

public class ShootAllBalls implements Command {
	private final Superstructure m_superstructure;
	private final Timer m_timer;
	private final Timer m_waitingForShotTimer;
	private boolean m_hasAttainedSpeed, m_intakeAndShoot;
	private final double m_timeout;

	public ShootAllBalls(final boolean intakeAndShoot, final double timeout) {
		m_superstructure = Superstructure.getInstance();
		m_timer = new Timer();
		m_waitingForShotTimer = new Timer();
		m_hasAttainedSpeed = false;
		m_timeout = timeout;
		m_intakeAndShoot = intakeAndShoot;
	}

	public ShootAllBalls() {
		this(false, 0.75);
	}

	@Override
	public void initialize() {
		m_superstructure.setWantedState(m_intakeAndShoot ? Superstructure.WantedState.SHOOTING_AND_INTAKING : Superstructure.WantedState.SHOOTING);
		m_timer.reset();
		m_timer.start();
	}

	@Override
	public void execute() {
		if (!m_hasAttainedSpeed && m_superstructure.getSystemState() == Superstructure.SystemState.SHOOTING) {
			m_waitingForShotTimer.start();
			m_hasAttainedSpeed = true;
		}
	}

	@Override
	public boolean isFinished() {
		return m_waitingForShotTimer.hasElapsed(m_timeout);
	}

	@Override
	public Set<Subsystem> getRequirements() {
		return Set.of();
	}

	@Override
	public void end(final boolean interrupted) {
		m_superstructure.setWantedState(Superstructure.WantedState.IDLE);
		m_timer.stop();
		m_waitingForShotTimer.stop();
		m_waitingForShotTimer.reset();
	}
}
