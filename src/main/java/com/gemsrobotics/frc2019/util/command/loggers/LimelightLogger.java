package com.gemsrobotics.frc2019.util.command.loggers;

import com.gemsrobotics.frc2019.util.camera.Limelight;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightLogger extends Command {
	private final Limelight m_limelight;

	public LimelightLogger(final Limelight limelight) {
		m_limelight = limelight;
	}

	@Override
	public void execute() {
		SmartDashboard.putBoolean("Limelight has target", m_limelight.isTargetPresent());
		SmartDashboard.putNumber("Limelight X Offset", m_limelight.getOffsetHorizontal());
		SmartDashboard.putNumber("Limelight Y Offset", m_limelight.getOffsetVertical());
		SmartDashboard.putNumber("Limelight Area", m_limelight.getArea());
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
