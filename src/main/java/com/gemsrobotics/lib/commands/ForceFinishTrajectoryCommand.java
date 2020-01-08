package com.gemsrobotics.lib.commands;

import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDrive;
import edu.wpi.first.wpilibj.command.InstantCommand;

public final class ForceFinishTrajectoryCommand extends InstantCommand {
    private final DifferentialDrive<?> m_chassis;

    public ForceFinishTrajectoryCommand(final DifferentialDrive<?> chassis) {
        m_chassis = chassis;
    }

    @Override
    protected void initialize() {
        m_chassis.forceFinishTrajectory();
    }
}
