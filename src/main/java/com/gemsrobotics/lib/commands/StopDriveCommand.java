package com.gemsrobotics.lib.commands;

import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDrive;
import edu.wpi.first.wpilibj.command.InstantCommand;

public final class StopDriveCommand extends InstantCommand {
    private DifferentialDrive<?> m_chassis;

    public StopDriveCommand(final DifferentialDrive<?> chassis) {
        m_chassis = chassis;
    }

    @Override
    protected void initialize() {
        m_chassis.setDisabled();
    }
}
