package com.gemsrobotics.frc2022.subsystems.uptake;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

public class LimelightColorObserver implements CargoColorObserver {
	private static final double TARGET_AREA_THRESHOLD = 0.1;

	private final NetworkTableEntry m_isAliveEntry, m_areaEntry;

	public LimelightColorObserver() {
		final var table = NetworkTableInstance.getDefault().getTable("limelight-cargo");
		m_isAliveEntry = table.getEntry("tv");
		m_areaEntry = table.getEntry("ta");
	}

	@Override
	public DriverStation.Alliance getCargoAlliance() {
		if (m_isAliveEntry.getBoolean(false)
			&& m_areaEntry.getDouble(Double.NEGATIVE_INFINITY) > TARGET_AREA_THRESHOLD
		) {
			return DriverStation.Alliance.Blue;
		} else {
			return DriverStation.Alliance.Red;
		}
	}

	@Override
	public boolean isCargoOurs() {
		final var alliance = DriverStation.getAlliance();
		return alliance == DriverStation.Alliance.Invalid || alliance == DriverStation.getAlliance();
	}
}
