package com.gemsrobotics.frc2022;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import java.util.Objects;

public class PneumaticsContainer {
	private static PneumaticsContainer INSTANCE;

	public static PneumaticsContainer getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new PneumaticsContainer();
		}

		return INSTANCE;
	}

	private final PneumaticsControlModule m_ph;

	private final DoubleSolenoid m_intake;
	private final DoubleSolenoid m_intake2;
	private final DoubleSolenoid m_swing;

	private PneumaticsContainer() {
		m_ph = new PneumaticsControlModule(0);
		m_intake = m_ph.makeDoubleSolenoid(0, 1);
		m_intake2 = m_ph.makeDoubleSolenoid(4, 5);
		m_swing = m_ph.makeDoubleSolenoid(2, 3);

		// m_ph = new PneumaticsControlModule(0);
		// m_intake = m_ph.makeDoubleSolenoid(8, 9);
		// m_intake2 = m_ph.makeDoubleSolenoid(0, 1);
		// m_swing = m_ph.makeDoubleSolenoid(11, 12);
	}

	public PneumaticsControlModule getPneumaticHub() {
		return m_ph;
	}

	public DoubleSolenoid getIntakeSolenoid() {
		return m_intake;
	}

	public DoubleSolenoid getIntakeSolenoid2() {
		return m_intake2;
	}

	public DoubleSolenoid getSwingSolenoid() {
		return m_swing;
	}
}
