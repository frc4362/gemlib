package com.gemsrobotics.frc2019.subsystems.inventory;

import edu.wpi.first.wpilibj.AnalogInput;

public class ReflectiveInventory extends Inventory {
	private static final double DETECTION_THRESHOLD = 0.1;

	private final AnalogInput m_reflectiveSensor;

	public ReflectiveInventory(final int port) {
		m_reflectiveSensor = new AnalogInput(port);
	}

	public double getRawSensor() {
		return m_reflectiveSensor.getAverageVoltage();
	}

	@Override
	public boolean hasPanel() {
		return !hasCargo();
	}

	@Override
	public boolean hasCargo() {
		return getRawSensor() > DETECTION_THRESHOLD;
	}
}
