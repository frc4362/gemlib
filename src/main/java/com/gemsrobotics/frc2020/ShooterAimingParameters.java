package com.gemsrobotics.frc2020;

import com.gemsrobotics.lib.math.se2.Rotation;

public final class ShooterAimingParameters {
	private boolean m_hoodDeployed;
	private double m_targetShooterRPM;
	private Rotation m_targetTurretRotation;

	public ShooterAimingParameters(
			final boolean hoodDeployed,
			final double targetShooterRPM,
			final Rotation targetTurretRotation
	) {
		m_hoodDeployed = hoodDeployed;
		m_targetShooterRPM = targetShooterRPM;
		m_targetTurretRotation = targetTurretRotation;
	}

	public boolean isHoodDeployed() {
		return m_hoodDeployed;
	}

	public double getShooterRPM() {
		return m_targetShooterRPM;
	}

	public Rotation getTurretRotation() {
		return m_targetTurretRotation;
	}
}
