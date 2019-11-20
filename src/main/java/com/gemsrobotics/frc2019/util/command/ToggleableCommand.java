package com.gemsrobotics.frc2019.util.command;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Represents a {@link Command} which may be turned on and off,
 * in a purely semantic fashion, with a single ending condition implemented
 * in {@link Command#isFinished()}, and will have different values for each
 */
@SuppressWarnings({"unused", "WeakerAccess"})
public abstract class ToggleableCommand extends Command {
	/**
	 * The starting mode for the {@link ToggleableCommand}.
	 * Note that the DEFAULT start mode may be changed.
	 */
	protected enum StartMode {
		ENABLED(true), DISABLED(false), DEFAULT(true);
		
		private final boolean m_mode;
		
		StartMode(final boolean b) {
			m_mode = b;
		}
		
		public boolean toBoolean() {
			return m_mode;
		}
	}
	
	private boolean m_enabled;

	@SuppressWarnings("SameParameterValue")
	protected ToggleableCommand(final StartMode startEnabled) {
		m_enabled = startEnabled.toBoolean();
	}

	protected ToggleableCommand() {
		this(StartMode.ENABLED);
	}

	/**
	 * Turns the command on and off
	 */
	public final void setEnabled(final boolean enabled) {
		m_enabled = enabled;
	}

	public final void enable() {
		setEnabled(true);
	}

	public final void disable() {
		setEnabled(false);
	}

	public final boolean isEnabled() {
		return m_enabled;
	}
	
	public final boolean isDisabled() {
		return !isEnabled();
	}

	protected abstract void whenEnabled();
	
	@SuppressWarnings("EmptyMethod")
	protected void whenDisabled() {	}

	@Override
	protected final void execute() {
		if (isEnabled()) {
			whenEnabled();
		} else {
			whenDisabled();
		}
	}
}
