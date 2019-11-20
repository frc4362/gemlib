package com.gemsrobotics.frc2019.commands;

import com.gemsrobotics.frc2019.commands.any.Wait;
import com.gemsrobotics.frc2019.subsystems.manipulator.Manipulator;
import edu.wpi.first.wpilibj.command.CommandGroup;

import static com.gemsrobotics.frc2019.util.command.Commands.commandOf;

public class PickupSequence extends CommandGroup {
	public PickupSequence(final Manipulator manipulator) {
		addSequential(commandOf(() -> {
			manipulator.getArm().set(true);
			manipulator.getHand().set(true);
		}));
		addSequential(new Wait(200));
		addSequential(commandOf(() -> {
			manipulator.getArm().set(true);
			manipulator.getHand().set(false);
		}));
		addSequential(new Wait(200));
		addSequential(commandOf(() -> {
			manipulator.getArm().set(false);
			manipulator.getHand().set(false);
		}));
	}
}
