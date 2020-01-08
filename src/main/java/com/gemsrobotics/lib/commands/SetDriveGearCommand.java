package com.gemsrobotics.lib.commands;

import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDrive;
import edu.wpi.first.wpilibj.command.InstantCommand;

public final class SetDriveGearCommand extends InstantCommand {
    private final DifferentialDrive<?> m_chassis;
    private final boolean m_wantsHighGear;

    public SetDriveGearCommand(final DifferentialDrive<?> chassis, final boolean wantsHighGear) {
        m_chassis = chassis;
        m_wantsHighGear = wantsHighGear;
    }

    @Override
    protected void initialize() {
        m_chassis.setHighGear(m_wantsHighGear);
    }
}
