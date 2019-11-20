package com.gemsrobotics.lib.commands;

import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.physics.DriveCharacterizer;
import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDrive;
import edu.wpi.first.wpilibj.command.Command;

import java.util.List;

import static java.lang.Math.abs;

public final class CollectDriveVelocityData extends Command {
    private static final double MAX_POWER = 0.25;
    private static final double RAMP_RATE = 0.02;

    private final DifferentialDrive m_chassis;
    private final List<DriveCharacterizer.VelocityDataPoint> m_data;
    private boolean isFinished = false;

    private double m_power;

    /**
     * @param chassis  the drive base to characterize
     * @param data     reference to the list where data points should be stored
     */
    public CollectDriveVelocityData(final DifferentialDrive chassis, final List<DriveCharacterizer.VelocityDataPoint> data) {
        requires(chassis);
        m_chassis = chassis;
        m_data = data;
    }

    @Override
    public void initialize() {
        m_power = 0.0;
    }

    @Override
    public void execute() {
        m_power = RAMP_RATE * timeSinceInitialized();
        m_chassis.setDriverControl(m_power, 0, false);

        final var velocities = m_chassis.getWheelProperty(MotorController::getVelocityAngular);
        m_data.add(new DriveCharacterizer.VelocityDataPoint(abs(velocities.left) + abs(velocities.right), m_power * 12.0));
    }

    @Override
    public boolean isFinished() {
        return m_power > MAX_POWER;
    }
}
