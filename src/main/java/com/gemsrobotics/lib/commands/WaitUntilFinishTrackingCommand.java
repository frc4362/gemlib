package com.gemsrobotics.lib.commands;

import edu.wpi.first.wpilibj.command.Command;
import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDrive;

public final class WaitUntilFinishTrackingCommand extends Command {
    private final DifferentialDrive m_chassis;

    public WaitUntilFinishTrackingCommand(final DifferentialDrive chassis) {
        m_chassis = chassis;
    }

    @Override
    public boolean isFinished() {
        return m_chassis.isTrajectoryFinished();
    }
}
