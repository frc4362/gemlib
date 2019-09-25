package com.gemsrobotics.lib.commands;

import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.physics.DriveCharacterizer;
import com.gemsrobotics.lib.utils.MathUtils;
import edu.wpi.first.wpilibj.command.Command;
import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDrive;

import java.util.List;

import static java.lang.Math.abs;

public class CollectDriveAccelerationData extends Command {
    private static final double POWER = 0.5;
    private static final double TOTAL_TIME = 2.0; //how long to run the test for

    private final DifferentialDrive m_chassis;
    private final List<DriveCharacterizer.AccelerationDataPoint> m_data;

    private double m_lastTime, m_lastVelocity;

    /**
     * @param data     reference to the list where data points should be stored=
     */
    public CollectDriveAccelerationData(final DifferentialDrive chassis, final List<DriveCharacterizer.AccelerationDataPoint> data) {
        m_chassis = chassis;
        m_data = data;
    }

    @Override
    public void initialize() {
        m_lastTime = Double.NaN;
        m_lastVelocity = Double.NaN;
        m_chassis.setOpenLoop(POWER, 0, false);
    }

    @Override
    public void execute() {
        final var velocities = m_chassis.getWheelProperty(MotorController::getVelocityAngular);
        final var currentVelocity = abs(velocities.left) + abs(velocities.right);

        //don't calculate acceleration until we've populated prevTime and prevVelocity
        if (Double.isNaN(m_lastTime)) {
            m_lastTime = timeSinceInitialized();
            m_lastVelocity = currentVelocity;
            return;
        }

        final var currentTime = timeSinceInitialized();
        final double acceleration = (currentVelocity - m_lastVelocity) / (currentTime - m_lastTime);

        //ignore accelerations that are too small
        if (acceleration > MathUtils.kEpsilon) {
            m_data.add(new DriveCharacterizer.AccelerationDataPoint(currentVelocity, POWER * 12.0, acceleration));
        }

        m_lastTime = currentTime;
        m_lastVelocity = currentVelocity;
    }

    @Override
    public boolean isFinished() {
        return timeSinceInitialized() > TOTAL_TIME;
    }
}
