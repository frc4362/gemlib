package com.gemsrobotics.frc2019.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;

public class ClimberRollerListener extends Command {
	private WPI_TalonSRX m_rollers;
	private XboxController m_controller;

	private boolean m_hasRun;

	public ClimberRollerListener(
			final WPI_TalonSRX rollers,
			final XboxController controller
	) {
		m_hasRun = false;

		m_rollers = rollers;
		m_controller = controller;
	}

	private static double deadband(final double val) {
		return Math.abs(val) > 0.2 ? val : 0.0;
	}

	@Override
	public void initialize() {
		m_hasRun = true;
	}

	@Override
	public void execute() {
		m_rollers.set(deadband(m_controller.getY(GenericHID.Hand.kLeft)));
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	public void reset() {
		m_hasRun = false;
	}

	public boolean hasPreviouslyRun() {
		return m_hasRun;
	}
}
