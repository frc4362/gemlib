package com.gemsrobotics.frc2019scuff;

import com.gemsrobotics.lib.drivers.hid.Gemstick;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import edu.wpi.first.wpilibj.TimedRobot;

public class ScuffedRobot extends TimedRobot {
    private Gemstick m_leftStick, m_rightStick;
    private MotorController m_frontLeft, m_frontRight, m_backLeft, m_backRight;

    @Override
    public void robotInit() {
        m_leftStick = new Gemstick(0);
        m_rightStick = new Gemstick(1);

        m_frontLeft = MotorControllerFactory.createDefaultTalonSRX(0);
        m_frontLeft.setInvertedOutput(false);
        m_backLeft = MotorControllerFactory.createDefaultTalonSRX(1);
        m_backLeft.follow(m_frontLeft, false);
        m_frontRight = MotorControllerFactory.createDefaultTalonSRX(2);
        m_frontRight.setInvertedOutput(true);
        m_backRight = MotorControllerFactory.createDefaultTalonSRX(3);
        m_backRight.follow(m_frontRight, false);
    }

    @Override
    public void teleopPeriodic() {
        final var leftVoltage = m_leftStick.getFrame().y();
        final var rightVoltage = m_rightStick.getFrame().y();

        m_frontLeft.setVoltage(leftVoltage);
        m_frontRight.setVoltage(rightVoltage);
    }
}
