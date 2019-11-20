package com.gemsrobotics.frc2018;

import com.gemsrobotics.frc2018.subsystems.Carriage;
import com.gemsrobotics.frc2018.subsystems.Chassis;
import com.gemsrobotics.frc2018.subsystems.Elevator;
import com.gemsrobotics.lib.commands.CurvatureDriveCommand;
import com.gemsrobotics.lib.drivers.hid.Gempad;
import com.gemsrobotics.lib.drivers.hid.Gemstick;
import com.gemsrobotics.lib.structure.SubsystemManager;
import com.gemsrobotics.lib.telemetry.Pod;
import com.gemsrobotics.lib.telemetry.reporting.ConsoleReporter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;

public final class Robot extends TimedRobot implements Loggable {
    private SubsystemManager m_subsystemManager;
    private Gemstick m_stickLeft, m_stickRight;
    private Gempad m_controller;

    @Override
    public void robotInit() {
        Pod.configure()
           .withoutMonitoring()
           .withReporters(ConsoleReporter.getInstance())
           .wake();

        m_subsystemManager = new SubsystemManager(
                Chassis.getInstance(),
                Carriage.getInstance(),
                Elevator.getInstance()
        );

        m_stickLeft = new Gemstick(0);
        m_stickRight = new Gemstick(1);
        m_controller = new Gempad(2);

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

        Scheduler.getInstance().add(
                new CurvatureDriveCommand(
                    Chassis.getInstance(),
                    m_stickRight::y,
                    m_stickRight::x,
                    m_stickRight::getTrigger)
        );
    }

    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();

        if (m_controller.getButton(Gempad.Button.B).getRisingEdge()) {
            Carriage.getInstance().setWantedMouthState(Carriage.MouthState.FORCE_CLOSE);
        } else if (m_controller.getButton(Gempad.Button.B).getFallingEdge()) {
            Carriage.getInstance().setWantedMouthState(Carriage.MouthState.AUTOMATIC);
        }

        Carriage.getInstance().setOpenLoop(m_controller.getTriggerValue(GenericHID.Hand.kLeft) - m_controller.getTriggerValue(GenericHID.Hand.kRight));
    }

    @Override
    public void disabledInit() {
        m_subsystemManager.disable();
    }
}
