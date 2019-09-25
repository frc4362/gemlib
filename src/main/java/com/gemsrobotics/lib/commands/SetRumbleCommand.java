package com.gemsrobotics.lib.commands;

import com.gemsrobotics.lib.drivers.hid.Gempad;
import edu.wpi.first.wpilibj.command.InstantCommand;

public final class SetRumbleCommand extends InstantCommand {
    private final Gempad m_controller;
    private final Gempad.RumbleKind m_kind;
    private final double m_strength;

    public SetRumbleCommand(final Gempad pad, final Gempad.RumbleKind kind, final double strength) {
        m_controller = pad;
        m_kind = kind;
        m_strength = strength;
    }

    public SetRumbleCommand(final Gempad pad, final double strength) {
        this(pad, Gempad.RumbleKind.ALL, strength);
    }

    @Override
    protected void initialize() {
        m_controller.setRumble(m_kind, m_strength);
    }
}
