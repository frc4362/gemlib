package com.gemsrobotics.lib.drivers.motorcontrol;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;

public final class MotorControllerGroup<T> {
    private final MotorController<T> m_master;
    private final List<MotorController<T>> m_slaves;

    public MotorControllerGroup(final MotorController<T> master, final List<MotorController<T>> slaves) {
        m_master = master;
        m_slaves = slaves;
    }

    public MotorController<T> getMaster() {
        return m_master;
    }

    public List<MotorController<T>> getSlaves() {
        return m_slaves;
    }

    public void forEach(final Consumer<MotorController<T>> action) {
        action.accept(m_master);
        m_slaves.forEach(action);
    }

    public boolean forEachAttempt(final Function<MotorController<T>, Boolean> action) {
        boolean success = true;

        success &= action.apply(m_master);

        for (final MotorController<T> slave : m_slaves) {
            success &= action.apply(slave);
        }

        return success;
    }

    public void followMaster(final boolean invert) {
        m_slaves.forEach(slave -> slave.follow(m_master, invert));
    }

    public void setSafe() {
        forEach(MotorController::setNeutral);
    }
}
