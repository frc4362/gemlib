package com.gemsrobotics.lib.commands;

import com.gemsrobotics.lib.drivers.hid.Gempad;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class DoRumbleCommand extends CommandGroup {
    public DoRumbleCommand(final Gempad gempad, final Gempad.RumbleKind kind, final double strength, final double duration) {
        addSequential(new SetRumbleCommand(gempad, kind, strength));
        addSequential(new WaitCommand(duration));
        addSequential(new SetRumbleCommand(gempad, Gempad.RumbleKind.ALL, 0.0));
    }
}
