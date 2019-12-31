package com.gemsrobotics.frc2019.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.gemsrobotics.lib.commands.EndlessCommand;
import com.gemsrobotics.lib.drivers.hid.Gempad;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;

public class ClimberRollerListener extends EndlessCommand {
	private TalonSRX m_rollers;
	private Gempad m_controller;

	private boolean m_hasRun;

	public ClimberRollerListener(
			final TalonSRX rollers,
			final Gempad controller
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
		m_rollers.set(ControlMode.PercentOutput, deadband(m_controller.getStick(GenericHID.Hand.kLeft).getTranslation().y()));
	}

	public void reset() {
		m_hasRun = false;
	}

	public boolean hasPreviouslyRun() {
		return m_hasRun;
	}
}
