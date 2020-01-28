package com.gemsrobotics.lib.commands;

import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.physics.Characterizer;
import com.gemsrobotics.lib.subsystems.drivetrain.ChassisState;
import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDrive;
import com.gemsrobotics.lib.subsystems.drivetrain.WheelState;
import edu.wpi.first.wpilibj.command.Command;

import java.util.List;

import static java.lang.Math.abs;

public final class CollectDriveVelocityData extends Command {
    private static final double MAX_POWER = 0.25;
    private static final double RAMP_RATE = 0.02;

    private final DifferentialDrive<?> m_chassis;
    private final List<Characterizer.VelocityDataPoint> m_data;

    private double m_power;

    /**
     * @param chassis  the drive base to characterize
     * @param data     reference to the list where data points should be stored
     */
    public CollectDriveVelocityData(final DifferentialDrive<?> chassis, final List<Characterizer.VelocityDataPoint> data) {
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
        m_chassis.setOpenLoop(new ChassisState(m_power, 0));

        final WheelState wheelVoltages = m_chassis.getWheelProperty(MotorController::getVoltageInput);
        final double systemVoltage = (wheelVoltages.left + wheelVoltages.right) / 2.0;

        final var velocities = m_chassis.getWheelProperty(MotorController::getVelocityAngularRadiansPerSecond);
        m_data.add(new Characterizer.VelocityDataPoint((abs(velocities.left) + abs(velocities.right)) / 2.0, m_power * systemVoltage));
    }

    @Override
    public boolean isFinished() {
        return m_power > MAX_POWER;
    }

    @Override
    public void end() {
        m_chassis.setOpenLoop(new ChassisState());
    }
}
