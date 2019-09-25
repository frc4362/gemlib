package com.gemsrobotics.lib.utils;

import java.util.Arrays;
import java.util.Collection;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Scheduler;

/**
 * A utility class for the creation and composition of {@link Command}s
 * and {@link CommandGroup}s
 */
@SuppressWarnings({"unused", "WeakerAccess"})
public final class CommandUtils {
	private CommandUtils() {
	}

	/**
	 * Turns an instance of {@link Runnable} into a one-time execution command.
	 * Reduces the amount of nice classes and {@link Command}s that need to be made
	 * @param action The {@link Runnable} to be turned into an {@link InstantCommand}
	 * @return The instantiated version of the passed {@link Runnable}
	 */
	public static InstantCommand commandOf(final Runnable action) {
		return new InstantCommand() {
			// FredBoat is with u
			// EDIT 10/26: ohhhhhh it's a joke about the word protected
			protected void initialize() {
				action.run();
			}
		};
	}

	/**
	 * Pass {@link Command} to be used to make a quick {@link CommandGroup}
	 * Generally reduces top-level clutter.
	 * @param actions The {@link Command}s to be made into a command group
	 */
	public static CommandGroup commandGroupOf(final Collection<Object> actions) {
		final var validActions = actions.stream().flatMap(action -> {
			if (action instanceof Command) {
				return Optional.of((Command) action).stream();
			} else if (action instanceof Runnable) {
				return Optional.of(commandOf((Runnable) action)).stream();
			} else {
				return Optional.<Command>empty().stream();
			}
		});

		return new CommandGroup() {
			{
				validActions.forEach(this::addSequential);
			}
		};
	}

	public static CommandGroup commandGroupOf(final Object... actions) {
		return commandGroupOf(Arrays.asList(actions));
	}

	/**
	 * @return A useless, do-nothing command for composition and a default value in various places
	 */
	public static InstantCommand nullCommand() {
		return commandOf(() -> {});
	}

	public static Command waitForState(final Supplier<Boolean> stateAchieved, final Command andThen) {
		return new Command() {
			@Override
			public boolean isFinished() {
				return stateAchieved.get();
			}

			@Override
			public void end() {
				Scheduler.getInstance().add(andThen);
			}
		};
	}

	public static Trigger triggerOf(final Supplier<Boolean> supplier) {
	    return new Trigger() {
            @Override
            public boolean get() {
                return supplier.get();
            }
        };
    }
}
