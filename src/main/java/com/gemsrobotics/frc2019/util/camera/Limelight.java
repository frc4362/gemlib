package com.gemsrobotics.frc2019.util.camera;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.Optional;

import static java.lang.Math.toRadians;

@SuppressWarnings("unused")
public class Limelight {
	private final NetworkTable m_table;

	private final NetworkTableEntry
			m_presentEntry, m_offsetHorizontalEntry, m_offsetVerticalEntry,
			m_areaEntry, m_modeLedEntry, m_modeCameraEntry, m_pipelineEntry,
			m_translationEntry;

	public Limelight() {
		m_table = NetworkTableInstance.getDefault().getTable("limelight");
		m_presentEntry = m_table.getEntry("tv");
		m_offsetHorizontalEntry = m_table.getEntry("tx");
		m_offsetVerticalEntry = m_table.getEntry("ty");
		m_areaEntry = m_table.getEntry("ta");
		m_modeLedEntry = m_table.getEntry("ledMode");
		m_modeCameraEntry = m_table.getEntry("cameraMode");
		m_pipelineEntry = m_table.getEntry("pipeline");

		m_translationEntry = m_table.getEntry("camtran");
	}

	public Optional<Double> getRawProperty(final String property) {
		final var val = m_table.getEntry(property).getDouble(999);
		return Optional.ofNullable(val == 999 ? null : val);
	}

	public enum LEDMode {
		DEFER(0),
		OFF(1),
		BLINK(2),
		ON(3);

		public final int value;

		LEDMode(final int v) {
			value = v;
		}
	}

	public enum CameraMode {
		CV(0),
		DRIVER(1);

		public final int value;

		CameraMode(final int v) {
			value = v;
		}
	}

	public boolean isTargetPresent() {
		return m_presentEntry.getDouble(0) == 1.0;
	}

	public double getOffsetHorizontal() {
		return toRadians(m_offsetHorizontalEntry.getDouble(0));
	}

	public double getOffsetVertical() {
		return toRadians(m_offsetVerticalEntry.getDouble(0));
	}

	public double getArea() {
		return m_areaEntry.getDouble(0.0);
	}

	public void setLEDMode(final LEDMode mode) {
		m_modeLedEntry.setDouble(mode.value);
	}

	public void setCameraMode(final CameraMode mode) {
		m_modeCameraEntry.setDouble(mode.value);
	}

	public void setPipeline(final int p) {
		if (p < 9 && p > 0) {
			m_pipelineEntry.setDouble(p);
		}
	}
}
