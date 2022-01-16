package com.gemsrobotics.lib.commands;

import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.physics.Characterizer;
import com.gemsrobotics.lib.subsystems.drivetrain.ChassisState;
import com.gemsrobotics.lib.subsystems.drivetrain.WheelState;
import com.gemsrobotics.lib.utils.MathUtils;
import edu.wpi.first.wpilibj.command.Command;
import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDrive;

import java.util.List;

import static java.lang.Math.abs;

public class CollectDriveAccelerationData extends Command {
    private static final double POWER = 0.5;
    private static final double TOTAL_TIME = 2.0; //how long to run the test for

    private final DifferentialDrive<?> m_chassis;
    private final List<Characterizer.AccelerationDataPoint> m_data;

    private double m_lastTime, m_lastVelocity;

    /**
     * @param data     reference to the list where data points should be stored
     */
    public CollectDriveAccelerationData(final DifferentialDrive<?> chassis, final List<Characterizer.AccelerationDataPoint> data) {
        m_chassis = chassis;
        m_data = data;
    }

    @Override
    public void initialize() {
        m_lastTime = Double.NaN;
        m_lastVelocity = Double.NaN;
        m_chassis.setOpenLoop(new ChassisState(POWER, 0.0));
    }

    @Override
    public void execute() {
        final var velocities = m_chassis.getWheelProperty(MotorController::getVelocityAngularRadiansPerSecond);
        final var currentVelocity = (abs(velocities.left) + abs(velocities.right)) / 2.0;

        //don't calculate acceleration until we've populated prevTime and prevVelocity
        if (Double.isNaN(m_lastTime)) {
            m_lastTime = timeSinceInitialized();
            m_lastVelocity = currentVelocity;
            return;
        }

        final var currentTime = timeSinceInitialized();
        final double acceleration = (currentVelocity - m_lastVelocity) / (currentTime - m_lastTime);

        final WheelState wheelVoltages = m_chassis.getWheelProperty(MotorController::getVoltageOutput);
        final double averageVoltage = (wheelVoltages.left + wheelVoltages.right) / 2.0;

        //ignore accelerations that are too small
        if (acceleration > MathUtils.Epsilon) {
            m_data.add(new Characterizer.AccelerationDataPoint(currentVelocity, averageVoltage, acceleration));
        }

        m_lastTime = currentTime;
        m_lastVelocity = currentVelocity;
    }

    @Override
    public boolean isFinished() {
        return timeSinceInitialized() > TOTAL_TIME;
    }

    @Override
    public void end() {
        m_chassis.setOpenLoop(new ChassisState());
    }
}
