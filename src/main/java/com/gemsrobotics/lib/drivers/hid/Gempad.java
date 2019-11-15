package com.gemsrobotics.lib.drivers.hid;

import com.gemsrobotics.lib.commands.DoRumbleCommand;
import com.gemsrobotics.lib.commands.SetRumbleCommand;
import com.gemsrobotics.lib.commands.WaitCommand;
import com.gemsrobotics.lib.data.DigitalSignalTrigger;
import com.gemsrobotics.lib.data.ThresholdSignal;
import com.gemsrobotics.lib.math.se2.Translation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.POVButton;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

import java.util.*;

import static com.gemsrobotics.lib.utils.CommandUtils.triggerOf;

public final class Gempad {
    private static final double DEFAULT_DEADBAND = 0.3;

    public enum RumbleKind {
        LEFT,
        RIGHT,
        ALL
    }

    // the order is VERY IMPORTANT
    public enum Button {
        A,
        B,
        X,
        Y,
        BUMPER_LEFT,
        BUMPER_RIGHT,
        BACK,
        START,
        STICK_LEFT,
        STICK_RIGHT;

        public static final int COUNT = values().length;
    }

    private class ButtonListener extends DigitalSignalTrigger {
        private final int m_buttonId;

        private ButtonListener(final int buttonId) {
            m_buttonId = buttonId;
        }

        @Override
        public final boolean get() {
            return m_internal.getRawButton(m_buttonId);
        }

        @Override
        public final boolean getRisingEdge() {
            return m_internal.getRawButtonPressed(m_buttonId);
        }

        @Override
        public final boolean getFallingEdge() {
            return m_internal.getRawButtonReleased(m_buttonId);
        }

        @Override
        public final void onRisingEdge(final Command command) {
            m_triggersRisingEdge.computeIfAbsent(m_buttonId, id -> triggerOf(ButtonListener.this::getRisingEdge)).whenActive(command);
        }

        @Override
        public final void onFallingEdge(final Command command) {
            m_triggersFallingEdge.computeIfAbsent(m_buttonId, id -> triggerOf(ButtonListener.this::getFallingEdge)).whenActive(command);
        }
    }

    private final XboxController m_internal;
    private final List<ButtonListener> m_buttons;
    private final Map<Integer, Trigger> m_triggersRisingEdge;
    private final Map<Integer, Trigger> m_triggersFallingEdge;
    private double m_deadband;

    public Gempad(final int port) {
        m_internal = new XboxController(port);

        m_buttons = new ArrayList<>(Button.COUNT);
        m_triggersRisingEdge = new HashMap<>(Button.COUNT);
        m_triggersFallingEdge = new HashMap<>(Button.COUNT);
        m_deadband = 0.0;
    }

    public synchronized Gempad withRadialDeadband(final double deadband) {
        m_deadband = deadband;
        return this;
    }

    public synchronized Gempad withDefaultDeadband() {
        return withRadialDeadband(DEFAULT_DEADBAND);
    }

    public synchronized DigitalSignalTrigger getButton(final Button button) {
        final var idx = button.ordinal();

        if (Objects.isNull(m_buttons.get(button.ordinal()))) {
            m_buttons.set(idx, new ButtonListener(idx + 1)); // ports are 1-indexed internally
        }

        return m_buttons.get(idx);
    }

    public POVDirection getPOV() {
        return POVDirection.ofDegrees(m_internal.getPOV());
    }

    public POVButton getPOVButton(final POVDirection direction) {
        return new POVButton(m_internal, direction.getDegrees());
    }

    public synchronized void setRumble(final RumbleKind kind, final double strength) {
        switch (kind) {
            case LEFT:
                m_internal.setRumble(GenericHID.RumbleType.kLeftRumble, strength);
                break;
            case RIGHT:
                m_internal.setRumble(GenericHID.RumbleType.kRightRumble, strength);
                break;
            case ALL:
                m_internal.setRumble(GenericHID.RumbleType.kLeftRumble, strength);
                m_internal.setRumble(GenericHID.RumbleType.kRightRumble, strength);
                break;
        }
    }

    public void doRumble(final RumbleKind kind, final double strength, final double duration) {
        new DoRumbleCommand(this, kind, strength, duration).start();
    }

    public Translation getStick(final GenericHID.Hand side) {
        final var candidate = new Translation(m_internal.getX(side), m_internal.getY(side));

        if (candidate.norm() > m_deadband) {
            return candidate;
        } else {
            return Translation.identity();
        }
    }

    public double getTriggerValue(final GenericHID.Hand side) {
        return m_internal.getTriggerAxis(side);
    }

    public DigitalSignalTrigger getTriggerButton(final GenericHID.Hand side) {
        return new ThresholdSignal(() -> m_internal.getTriggerAxis(side), 0.9);
    }

    public final XboxController getInternals() {
        return m_internal;
    }
}
