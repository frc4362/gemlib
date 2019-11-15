package com.gemsrobotics.lib.commands;

import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDrive;
import edu.wpi.first.wpilibj.command.InstantCommand;

public final class SetDriveHeadingCommand extends InstantCommand {
    private final DifferentialDrive m_chassis;
    private final Rotation m_targetHeading;

    public SetDriveHeadingCommand(final DifferentialDrive chassis, final Rotation heading) {
        m_chassis = chassis;
        m_targetHeading = heading;
    }

    @Override
    public void initialize() {
        m_chassis.setHeading(m_targetHeading);
    }
}
