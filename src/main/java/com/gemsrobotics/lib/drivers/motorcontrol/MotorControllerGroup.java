package com.gemsrobotics.lib.drivers.motorcontrol;

import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;

public final class MotorControllerGroup {
    private final MotorController m_master;
    private final List<MotorController> m_slaves;

    public MotorControllerGroup(final MotorController master, final MotorController... slaves) {
        m_master = master;
        m_slaves = Arrays.asList(slaves);
    }

    public MotorController getMaster() {
        return m_master;
    }

    public List<MotorController> getSlaves() {
        return m_slaves;
    }

    public void forEach(final Consumer<MotorController> action) {
        action.accept(m_master);
        m_slaves.forEach(action);
    }

    public boolean forEachAttempt(final Function<MotorController, Boolean> action) {
        boolean success = true;

        success &= action.apply(m_master);

        for (final MotorController slave : m_slaves) {
            success &= action.apply(slave);
        }

        return success;
    }

    public void followMaster(final boolean invert) {
        m_slaves.forEach(slave -> slave.follow(m_master, invert));
    }

    public void setSafe() {
        forEach(motor -> motor.setVoltage(0.0));
    }
}
