package com.gemsrobotics.lib.data;

import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;

import java.util.function.Supplier;

import static com.gemsrobotics.lib.utils.CommandUtils.commandOf;

public abstract class DigitalSignalTrigger {
    public static DigitalSignalTrigger of(final Supplier<Boolean> getter) {
        return new DigitalSignalTrigger() {
            @Override
            public boolean get() {
                return getter.get();
            }
        };
    }

    private boolean m_lastValue = get();

    /**
     * Returns whether or not the trigger is active.
     *
     * <p>This method will be called repeatedly a command is linked to the Trigger.
     *
     * @return whether or not the trigger condition is active.
     */
    public abstract boolean get();

    public boolean getRisingEdge() {
        final var currentValue = get();
        final var ret = !m_lastValue && currentValue;
        m_lastValue = currentValue;
        return ret;
    }

    public boolean getFallingEdge() {
        final var currentValue = get();
        final var ret = m_lastValue && currentValue;
        m_lastValue = currentValue;
        return ret;
    }

    /**
     * Starts the given command whenever the trigger just becomes active.
     *
     * @param command the command to start
     */
    public void onRisingEdge(final Command command) {
        new Trigger.ButtonScheduler() {
            private boolean m_pressedLast = get();

            @Override
            public void execute() {
                boolean pressed = get();

                if (!m_pressedLast && pressed) {
                    command.start();
                }

                m_pressedLast = pressed;
            }
        }.start();
}

    public final void onRisingEdge(final Runnable runnable) {
        onRisingEdge(commandOf(runnable));
    }

    /**
     * Constantly starts the given command while the button is held.
     *
     * {@link Command#start()} will be called repeatedly while the trigger is active, and will be
     * canceled when the trigger becomes inactive.
     *
     * @param command the command to start
     */
    public void whileRisen(final InstantCommand command) {
        final var cmd = new Command() {
            @Override
            public void execute() {
                command.start();
            }

            @Override
            protected boolean isFinished() {
                return false;
            }
        };

        onRisingEdge(cmd);
        onFallingEdge(cmd::cancel);
    }

    public final void whileRisen(final Runnable runnable) {
        whileRisen(commandOf(runnable));
    }

    /**
     * Starts the command when the trigger becomes inactive.
     *
     * @param command the command to start
     */
    public void onFallingEdge(final Command command) {
        new Trigger.ButtonScheduler() {
            private boolean m_pressedLast = get();

            @Override
            public void execute() {
                boolean pressed = get();

                if (m_pressedLast && !pressed) {
                    command.start();
                }

                m_pressedLast = pressed;
            }
        }.start();
    }

    public final void onFallingEdge(final Runnable runnable) {
        onFallingEdge(commandOf(runnable));
    }

    /**
     * Toggles a command when the trigger becomes active.
     *
     * @param command the command to toggle
     */
    public void toggleOnRisingEdge(final Command command) {
        onRisingEdge(() -> {
            if (command.isRunning()) {
                command.cancel();
            } else {
                command.start();
            }
        });
    }

    public void toggleOnFallingEdge(final Command command) {
        onFallingEdge(() -> {
            if (command.isRunning()) {
                command.cancel();
            } else {
                command.start();
            }
        });
    }

    /**
     * Cancels a command when the trigger becomes active.
     *
     * @param command the command to cancel
     */
    public void cancelOnRisingEdge(final Command command) {
        onRisingEdge(command::cancel);
    }

    /**
     * Cancels a command when the trigger becomes inactive.
     *
     * @param command the command to cancel
     */
    public void cancelOnFallingEdge(final Command command) {
        onFallingEdge(command::cancel);
    }
}
