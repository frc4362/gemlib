package com.gemsrobotics.chairbot;

import com.gemsrobotics.lib.drivers.hid.Gemstick;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import edu.wpi.first.wpilibj.TimedRobot;

public class Chairbot extends TimedRobot {
    private Gemstick m_leftStick, m_rightStick;
    private MotorController m_motorLeft, m_motorRight;

    @Override
    public void robotInit() {
        m_leftStick = new Gemstick(0);
        m_rightStick = new Gemstick(1);

        m_motorLeft = MotorControllerFactory.createDefaultTalonSRX(0);
        m_motorLeft.setInvertedOutput(false);
        m_motorRight = MotorControllerFactory.createDefaultTalonSRX(2);
        m_motorRight.setInvertedOutput(true);

        MotorControllerFactory.createSlaveTalonSRX(1).follow(m_motorLeft, false);
        MotorControllerFactory.createSlaveTalonSRX(3).follow(m_motorRight, false);
    }

    @Override
    public void teleopPeriodic() {
        m_motorLeft.setDutyCycle(m_leftStick.y());
        m_motorRight.setDutyCycle(m_rightStick.y());
    }
}
