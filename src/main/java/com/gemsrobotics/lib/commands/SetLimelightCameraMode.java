package com.gemsrobotics.lib.commands;

import com.gemsrobotics.lib.subsystems.Limelight;
import edu.wpi.first.wpilibj.command.InstantCommand;

public final class SetLimelightCameraMode extends InstantCommand {
    private final Limelight m_limelight;
    private final Limelight.CameraMode m_mode;

    public SetLimelightCameraMode(final Limelight limelight, final Limelight.CameraMode cameraMode) {
        m_limelight = limelight;
        m_mode = cameraMode;
    }

    @Override
    protected void initialize() {
        m_limelight.setCameraMode(m_mode);
    }
}
