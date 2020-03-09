package com.gemsrobotics.frc2020.commands;

import com.gemsrobotics.lib.subsystems.drivetrain.ChassisState;
import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDrive;
import com.gemsrobotics.lib.timing.ElapsedTimer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public final class TimedDriveCommand extends Command {
	private final DifferentialDrive<?> m_drive;
	private final double m_speed;
	private ElapsedTimer m_timer;

	public TimedDriveCommand(final DifferentialDrive<?> drive, final double speed, final double time) {
		m_drive = drive;
		m_speed = speed;
		m_timer = new ElapsedTimer(time);
	}

	@Override
	public void initialize() {
		m_timer.reset();
		m_drive.setOpenLoop(new ChassisState(m_speed, 0.0));
	}

	@Override
	public boolean isFinished() {
		return m_timer.hasElapsed();
	}

	@Override
	public void end() {
		m_drive.setDisabled();
	}
}
