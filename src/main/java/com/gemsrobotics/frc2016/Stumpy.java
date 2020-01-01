package com.gemsrobotics.frc2016;

import com.gemsrobotics.lib.drivers.hid.Gempad;
import com.gemsrobotics.lib.drivers.hid.Gemstick;
import com.gemsrobotics.lib.structure.SubsystemManager;
import edu.wpi.first.wpilibj.TimedRobot;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;

public final class Stumpy extends TimedRobot implements Loggable {
	private SubsystemManager m_subsystems;
	private Gemstick m_leftStick, m_rightStick;
	private Gempad m_controller;

	@Override
	public void robotInit() {
		m_subsystems = new SubsystemManager(

		);

		m_leftStick = new Gemstick(0);
		m_rightStick = new Gemstick(1);
		m_controller = new Gempad(2);

		Logger.configureLogging(this);
		m_subsystems.init();
	}

	@Override
	public void teleopInit() {
		m_subsystems.enable();
	}

	@Override
	public void teleopPeriodic() {

	}

	@Override
	public void autonomousInit() {
		m_subsystems.enable();
	}

	@Override
	public void disabledInit() {
		m_subsystems.disable();
	}
}
