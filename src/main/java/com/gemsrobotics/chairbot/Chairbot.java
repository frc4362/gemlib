package com.gemsrobotics.chairbot;

import com.gemsrobotics.lib.drivers.hid.Gemstick;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import edu.wpi.first.wpilibj.TimedRobot;

public class Chairbot extends TimedRobot {
    private Gemstick m_stick;
    private MotorController m_motorLeft, m_motorRight;

    @Override
    public void robotInit() {
        m_stick = new Gemstick(0);

        m_motorLeft = MotorControllerFactory.createDefaultTalonSRX(0);
        m_motorLeft.setInvertedOutput(false);
        m_motorRight = MotorControllerFactory.createDefaultTalonSRX(2);
        m_motorRight.setInvertedOutput(true);

        MotorControllerFactory.createSlaveTalonSRX(1).follow(m_motorLeft, false);
        MotorControllerFactory.createSlaveTalonSRX(3).follow(m_motorRight, false);
    }

    @Override
    public void teleopPeriodic() {
        m_motorLeft.setDutyCycle(m_stick.y() - m_stick.x());
        m_motorRight.setDutyCycle(m_stick.y() + m_stick.x());
    }
}
