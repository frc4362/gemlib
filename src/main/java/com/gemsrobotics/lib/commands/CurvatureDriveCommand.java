package com.gemsrobotics.lib.commands;

import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDrive;

import java.util.function.Supplier;

public final class CurvatureDriveCommand extends EndlessCommand {
    private final DifferentialDrive m_chassis;
    private final Supplier<Double> m_throttle, m_wheel;
    private final Supplier<Boolean> m_isQuickturn;

    public CurvatureDriveCommand(
            final DifferentialDrive chassis,
            final Supplier<Double> throttle,
            final Supplier<Double> wheel,
            final Supplier<Boolean> quickturn
    ) {
        requires(chassis);
        m_chassis = chassis;
        m_throttle = throttle;
        m_wheel = wheel;
        m_isQuickturn = quickturn;
    }

    @Override
    public void execute() {
        m_chassis.setCurvatureDrive(m_throttle.get(), m_wheel.get(), m_isQuickturn.get());
    }
}
