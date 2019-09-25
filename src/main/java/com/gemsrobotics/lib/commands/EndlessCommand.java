package com.gemsrobotics.lib.commands;

import edu.wpi.first.wpilibj.command.Command;

public abstract class EndlessCommand extends Command {
	@Override
	public boolean isFinished() {
		return false;
	}
}
