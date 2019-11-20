package com.gemsrobotics.frc2019.subsystems.inventory;


import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;

@SuppressWarnings({"unused", "WeakerAccess"})
public class UltrasonicInventory extends Inventory {
	private final AnalogInput m_panelSensor1, m_panelSensor2;
	private final DigitalInput m_cargoSensor;

	private final double m_voltageMin, m_voltageMax;

	public UltrasonicInventory(final UltrasonicInventoryConfig cfg) {
		final int[] panelPorts = cfg.panelPorts;
		final int cargoPort = cfg.cargoPort;

		m_panelSensor1 = new AnalogInput(panelPorts[0]);
		m_panelSensor2 = new AnalogInput(panelPorts[1]);
		m_cargoSensor = new DigitalInput(cargoPort);

		m_voltageMin = cfg.voltageRange[0];
		m_voltageMax = cfg.voltageRange[1];
	}

	private boolean isValidPanelVoltage(final double voltage) {
		return voltage < m_voltageMax && voltage > m_voltageMin;
	}

	@Override
	public boolean hasPanel() {
		return isValidPanelVoltage(m_panelSensor1.getVoltage())
				|| isValidPanelVoltage(m_panelSensor2.getVoltage());
	}

	@Override
	public boolean hasCargo() {
		return !m_cargoSensor.get();
	}
}
