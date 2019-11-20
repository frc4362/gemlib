package com.gemsrobotics.frc2019.util.joy;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;

import com.gemsrobotics.frc2019.util.command.Commands;
import static com.gemsrobotics.frc2019.util.command.Commands.commandOf;

/**
 * It's times like these where I wish Java had a typedef
 *
 * Wrapper class for {@link JoystickButton} which allows for complex behavior
 * to be described monadically through composition of various predicates
 * @author Ethan
 */
@SuppressWarnings({"unused", "WeakerAccess"})
public final class Gembutton extends JoystickButton {
	/**
	 * Same constructor as the superclass {@link JoystickButton}
	 * @param joystick The stick to poll values from
	 * @param buttonNumber Which button to read from the joystick
	 */
	public Gembutton(final GenericHID joystick, final int buttonNumber) {
		super(joystick, buttonNumber);
	}

	/**
	 * Negates a boolean supplier
	 * @param base The {@link Supplier} to be inverted
	 * @return An inverted {@link Supplier}
	 */
	private static Supplier<Boolean> negated(final Supplier<Boolean> base) {
		return () -> !base.get();
	}

	/**
	 * The base method for all operations of {@link Gembutton}
	 * Every other kind of {@link JoystickButton#whenPressed(Command)}-like methods
	 * call back to this one.
	 * @param condition The base predicate which is used to discriminate at press-time
	 * @param action The action which will be carried out if the predicate proves true
	 * @param otherwise The action which will be carried out otherwise
	 */
	public void whenPressedIfElse(
			final Supplier<Boolean> condition, 
			final Runnable action, 
			final Runnable otherwise
	) {
		if (condition.get()) {
			this.whenPressed(action);
		} else {
			this.whenPressed(otherwise);
		}
	}

	/**
	 * Simpler version of {@link Gembutton#whenPressedIfElse(Supplier, Runnable, Runnable)}
	 * Doesn't have an else case
	 * @param condition Predicate to determine whether to carry out the action at press-time
	 * @param action The action that may or may not occur when a button is pressed
	 */
	public void whenPressedIf(final Supplier<Boolean> condition, final Runnable action) {
		this.whenPressedIfElse(condition, action, () -> {});
	}

	/**
	 * Simply a negated version of {@link Gembutton#whenPressedIf(Supplier, Runnable)}
	 * @param condition The predicate to be negated before it is considered
	 * @see Gembutton#whenPressedIf(Supplier, Runnable)
	 */
	public void whenPressedUnless(final Supplier<Boolean> condition, final Runnable action) {
		this.whenPressedIf(negated(condition), action);
	}

	/**
	 * More of an overload than an extension or override- allows one to bind a {@link Runnable}
	 * to a button instead of only an {@link Command}
	 * @param action The {@link Runnable} to be converted to a command before being bound
	 * @see Commands#commandOf(Runnable)
	 */
	public void whenPressed(final Runnable action) {
		super.whenPressed(commandOf(action));
	}

	/**
	 * The clone of {@link Gembutton#whenPressedIfElse(Supplier, Runnable, Runnable)}
	 * that is necessary for full functionality
	 * @see Gembutton#whenPressedIfElse(Supplier, Runnable, Runnable)
	 */
	public void whenReleasedIfElse(
			final Supplier<Boolean> condition,
			final Runnable action,
			final Runnable otherwise
	) {
		if (condition.get()) {
			this.whenReleased(action);
		} else {
			this.whenReleased(otherwise);
		}
	}

	/**
	 * Just another clone method
	 * @see Gembutton#whenPressedIfElse(Supplier, Runnable, Runnable)
	 */
	public void whenReleasedIf(final Supplier<Boolean> condition, final Runnable action) {
		this.whenReleasedIfElse(condition, action, () -> {});
	}

	/**
	 * Another inverted-clone-method
	 * @see Gembutton#whenPressedUnless(Supplier, Runnable)
	 */
	public void whenReleasedUnless(final Supplier<Boolean> condition, final Runnable action) {
		this.whenReleasedIf(negated(condition), action);
	}

	/**
	 * Wrapper method to allow {@link Runnable} to be used in place of a command
	 * @see Gembutton#whenPressed(Runnable)
	 * @see Commands#commandOf(Runnable)
	 */
	public void whenReleased(final Runnable action) {
		super.whenReleased(commandOf(action));
	}

	/**
	 * The base method to use constant-execution commands
	 * NOTE: No reason to ever use a theoretical whileReleasedIfElse
	 * @param condition The constant-check predicate
	 * @param action To be run while the button is held AND the condition proves true
	 * @param otherwise What is executed otherwise
	 */
	public void whileHeldIfElse(
			final Supplier<Boolean> condition,
			final Runnable action,
			final Runnable otherwise
	) {
		this.whileHeld(() -> {
			if (condition.get()) {
				action.run();
			} else {
				otherwise.run();
			}
		});
	}

	public void whileHeldIf(final Supplier<Boolean> condition, final Runnable action) {
		this.whileHeldIfElse(condition, action, () -> {});
	}

	public void whileHeld(final Runnable action) {
		super.whileHeld(commandOf(action));
	}

	public void setToggle(final Runnable toggle1, final Runnable toggle2) {
		final boolean[] val = { false };

		whenPressed(() -> {
			if (val[0]) {
				toggle1.run();
			} else {
				toggle2.run();
			}

			val[0] = !val[0];
		});
	}
}
