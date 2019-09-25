package com.gemsrobotics.lib.commands;

import com.gemsrobotics.lib.subsystems.Limelight;
import edu.wpi.first.wpilibj.command.InstantCommand;

public final class SetLimelightPipeline extends InstantCommand {
    private final Limelight m_limelight;
    private final int m_pipeline;

    public SetLimelightPipeline(final Limelight limelight, final int pipeline) {
        m_limelight = limelight;
        m_pipeline = pipeline;
    }

    @Override
    protected void initialize() {
        m_limelight.setSelectedPipeline(m_pipeline);
    }
}
