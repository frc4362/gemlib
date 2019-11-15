package com.gemsrobotics.lib.commands;

import edu.wpi.first.wpilibj.command.Command;

public abstract class EndlessCommand extends Command {
	@Override
	public final boolean isFinished() {
		return false;
	}
}
