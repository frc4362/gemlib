package com.gemsrobotics.frc2022;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;

import java.util.Objects;

public class PneumaticsContainer {
	private static PneumaticsContainer INSTANCE;

	public static PneumaticsContainer getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new PneumaticsContainer();
		}

		return INSTANCE;
	}

	private final PneumaticHub m_ph;

	private final DoubleSolenoid m_intake;
	private final DoubleSolenoid m_swing;

	private PneumaticsContainer() {
		m_ph = new PneumaticHub(1);
		m_intake = m_ph.makeDoubleSolenoid(8, 9);
		m_swing = m_ph.makeDoubleSolenoid(11, 12);
	}

	public PneumaticHub getPneumaticHub() {
		return m_ph;
	}

	public DoubleSolenoid getIntakeSolenoid() {
		return m_intake;
	}

	public DoubleSolenoid getSwingSolenoid() {
		return m_swing;
	}
}
