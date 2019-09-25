package com.gemsrobotics.frc2018;

import com.gemsrobotics.frc2018.subsystems.Carriage;
import com.gemsrobotics.frc2018.subsystems.Chassis;
import com.gemsrobotics.frc2018.subsystems.Elevator;
import com.gemsrobotics.lib.structure.SubsystemManager;
import com.gemsrobotics.lib.telemetry.Pod;
import com.gemsrobotics.lib.telemetry.reporting.ConsoleReporter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;

public final class Robot extends TimedRobot implements Loggable {
    private SubsystemManager m_subsystemManager;

    @Override
    public void robotInit() {
        Pod.configure().withoutMonitoring().withReporters(ConsoleReporter.getInstance()).finish();

        m_subsystemManager = new SubsystemManager(
                Chassis.getInstance(),
                Carriage.getInstance(),
                Elevator.getInstance()
        );

        m_subsystemManager.init();
        Logger.configureLogging(this);
    }

    @Override
    public void robotPeriodic() {
        LEDController.getInstance().ifPresent(LEDController::update);
        Logger.updateEntries();
    }

    @Override
    public void autonomousInit() {
        m_subsystemManager.enable();
    }

    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        m_subsystemManager.enable();
    }

    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        m_subsystemManager.disable();
    }
}
