package com.gemsrobotics.lib.commands;

import com.gemsrobotics.lib.timing.ElapsedTimer;
import edu.wpi.first.wpilibj.command.Command;

/**
 * A command used just to idle in auton
 * Common uses are coming to a stop, waiting for a closed-loop something to actuate,
 * or scuffedly spacing some complex command groups created
 * with {@link edu.wpi.first.wpilibj.command.CommandGroup#addParallel(Command)}
 */
public class WaitCommand extends Command {
	private final double m_duration;

    public WaitCommand(final double duration) {
    	m_duration = duration;
    }

    @Override
    protected boolean isFinished() {
        return timeSinceInitialized() > m_duration;
    }
}
