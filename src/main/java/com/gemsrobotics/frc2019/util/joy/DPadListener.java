package com.gemsrobotics.frc2019.util.joy;

import java.util.HashMap;
import java.util.Map;
import java.util.stream.IntStream;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import com.gemsrobotics.frc2019.util.command.Commands;

/**
 * Allows one to bind commands to the DPad on a controller
 */
@SuppressWarnings("serial")
public final class DPadListener extends Command {
	private final XboxController m_controller;
	private final Map<Integer, Command> m_bindings;
	private int m_lastPressed;

	/**
	 * @param input The angle returned by {@link XboxController#getPOV()}
	 * @return The direction on the DPad from the angle
	 */
	private static int angleToID(final int input) {
		return input == -1 ? input : input / 45;
	}

	/**
	 * North is up, the rest are the 8 orientations recognized by an {@link XboxController}
	 */
	public enum Direction {
		NORTH(0),
		NORTH_EAST(1),
		EAST(2),
		SOUTH_EAST(3),
		SOUTH(4),
		SOUTH_WEST(5),
		WEST(6),
		NORTH_WEST(7);
		
		private final int m_value;

		Direction(final int value) {
			m_value = value;
		}

		public int getValue() {
			return m_value;
		}
	}

	/**
	 * Internal constructor used to setPercent up the actual controls
	 * @param controller {@link XboxController} to be bound to
	 * @param bindings The commands to be bound to angles on the POV
	 */
	private DPadListener(
			final XboxController controller,
			final Map<Integer, Command> bindings
	) {
		m_controller = controller;
		m_bindings = new HashMap<>();
		IntStream.range(-1, 8).forEach(id ->
			m_bindings.put(id, bindings.getOrDefault(id, Commands.nullCommand())));
	}

	/**
	 * Builder method for an instance of a {@link DPadListener} which will provide
	 * up to 10 commands for any one single {@link XboxController}
	 * @param controller The controller to listen for inputs on
	 * @param bindings The commands to bind to given inputs
	 * @return The completely initialized instance of {@link DPadListener}
	 */
	public static DPadListener of(
			final XboxController controller,
			final Map<Direction, Command> bindings
	) {
		return new DPadListener(
			controller,
			new HashMap<Integer, Command>() {{
				// turns Direction into ints
				bindings.forEach((k, v) ->
					put(k.getValue(), v));
			}}
		);
	}

	/**
	 * Listens for inputs and begins executions for the desired commands
	 */
    protected void execute() {
    	final int pressed = m_controller.getPOV();

    	if (pressed != m_lastPressed) {
    		System.out.println("Recognized POV state change to POV(" + pressed + ")");
    		Scheduler.getInstance().add(m_bindings.get(angleToID(pressed)));
    	}
    	
    	m_lastPressed = pressed;
    }

	/**
	 * This makes it a lot easier to figure out what state the POV is in
	 * @param controller The controller to read POV from
	 * @param button The direction on the POV to test
	 * @return Whether or not the button is pressed
	 */
	public static boolean isPressed(final XboxController controller, final Direction button) {
		return angleToID(controller.getPOV()) == button.getValue();
	}

    protected boolean isFinished() {
        return false;
    }
}
