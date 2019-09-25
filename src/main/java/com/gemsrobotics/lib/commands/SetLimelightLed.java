package com.gemsrobotics.lib.commands;

import com.gemsrobotics.lib.subsystems.Limelight;
import edu.wpi.first.wpilibj.command.InstantCommand;

public final class SetLimelightLed extends InstantCommand {
    private final Limelight m_limelight;
    private final Limelight.LEDMode m_mode;

    public SetLimelightLed(final Limelight limelight, final Limelight.LEDMode mode) {
        m_limelight = limelight;
        m_mode = mode;
    }

    @Override
    protected void initialize() {
        m_limelight.setLEDMode(m_mode);
    }
}
