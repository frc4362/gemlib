package com.gemsrobotics.frc2019.commands.any;

import edu.wpi.first.wpilibj.command.Command;

/**
 * A command used just to idle in auton
 * Common uses are coming to a stop, waiting for a closed-loop something to actuate,
 * or properly spacing some complex command groups created
 * with {@link edu.wpi.first.wpilibj.command.CommandGroup#addParallel(Command)}
 */
@SuppressWarnings({"unused", "WeakerAccess"})
public class Wait extends Command {
	private final long m_length;
	private long m_endTime;

    public Wait(final long length) {
    	m_length = length;
    }

    protected void initialize() {
    	m_endTime = System.currentTimeMillis() + m_length;
    }

    protected boolean isFinished() {
        return m_endTime < System.currentTimeMillis();
    }
}
