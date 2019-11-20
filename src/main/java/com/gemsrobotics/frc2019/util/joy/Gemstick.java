package com.gemsrobotics.frc2019.util.joy;

import java.util.function.Function;

import com.gemsrobotics.frc2019.util.func.FunctionPipeline;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Over-engineered {@link Joystick} extension which has a LOT of behavior
 * Allows for POV controls, layered, sophisticated deadbanding on every axis,
 * and doesn't update values unless it needs to do so
 *
 * @author Ethan Malzone
 */
@SuppressWarnings({"unused", "WeakerAccess"})
public final class Gemstick extends Joystick {
	private static int instances = 0;

	// frame length in ms, determines the update rate of the joysticks
	// this might be wrong... uhHh
	// EDIT: 11/7 it was def wrong
	private static final int FRAME_LENGTH_MS = 20;

	/**
	 * Packages all values provided by the {@link Joystick} buttons in one frame
	 * so we update and store all values as we need them- not as they're available
	 */
	public enum Lens {
		X(JoystickFrame::getX), 
		Y(JoystickFrame::getY), 
		Z(JoystickFrame::getZ), 
		MAGNITUDE(JoystickFrame::getMagnitude), 
		AZIMUTH(JoystickFrame::getAzimuth);

		private final Function<JoystickFrame, Double> m_getter;

		Lens(final Function<JoystickFrame, Double> getter) {
			m_getter = getter;
		}

		public double apply(final JoystickFrame frame) {
			return m_getter.apply(frame);
		}
	}

	/**
	 * Similar to {@link com.gemsrobotics.frc2019.util.joy.DPadListener.Direction}
	 */
	public enum POVState {
		NONE(-1), 
		N(0), 
		NE(1), 
		E(2), 
		SE(3), 
		S(4), 
		SW(5), 
		W(6), 
		NW(7);

		private final int m_val;

		POVState(final int val) {
			m_val = val;
		}

		public int getValue() {
			return m_val;
		}

		public static POVState of(final int val) {
			switch (val) {
			case -1: return POVState.NONE;
			case 0: return POVState.N;
			case 1: return POVState.NE;
			case 2: return POVState.E;
			case 3: return POVState.SE;
			case 4: return POVState.S;
			case 5: return POVState.SW;
			case 6: return POVState.W;
			case 7: return POVState.NW;
			default:
				// this should never happen
				throw new RuntimeException("Invalid POVState state!");
			}
		}

		public static POVState ofDegrees(final int degrees) {
			return degrees == -1 ? NONE : of(degrees / 45);
		}

		public int toDegrees() {
			if (m_val == -1) {
				return -1;
			} else {
				return m_val * 45;
			}
		}
	}

	/**
	 * Provides sophisticated deadbanding utilities and options
	 * This is doable as it doesn't matter which order you apply these in
	 */
	@SuppressWarnings("unused")
	private static class DeadbandingOptions {
		private static final double TWO_THIRDS = 2.0 / 3.0;

		/**
		 * Scales a value that's from a given range to 1.0 in either direction,
		 * instead of from 0-1.00
		 * ie scale(0.75, 0.5) -> 0.5 motor output
		 * [1.0 - range, 1.0] -> [0.0, 1.0]
		 * @param input The number to be scaled
		 * @param range The amount by which the scale should be adjusted
		 * @return The appropriately scaled input
		 */
		private static double scale(final double input, final double range) {
			return (1.0 / range) * (input - ((1.0 - range) * Math.signum(input)));
		}

		// fundamental deadbanding method
		private static double deadband(final double input, final double threshold) {
			return Math.abs(input) > threshold ? scale(input, 1 - threshold) : 0;
		}

		/**
		 * Deadbands the {@link Gemstick} in a rectangle shape
		 * @param limitX The width of the rectangle
		 * @param limitY The height of the rectangle
		 * @return The modified pipeline which will provide this
		 */
		public static Function<JoystickFrame, JoystickFrame> makeRectangleDeadband(
				final double limitX, final double limitY
		) {
			return (stick) -> {
				final double x = stick.getX(), 
						     y = stick.getY();

				return new JoystickFrame(
						Math.abs(x) > limitX ? scale(x, 1 - limitX) : 0,
						Math.abs(y) > limitY ? scale(y, 1 - limitY) : 0,
						stick.getZ()
				);
			};
		}

		/**
		 * Makes a square-shaped deadband
		 * @param dim The width and height of the square
		 * @return The function which will deadband values in a square
		 * @see DeadbandingOptions#makeRectangleDeadband(double, double)
		 */
		public static Function<JoystickFrame, JoystickFrame> makeSquareDeadband(final double dim) {
			return makeRectangleDeadband(dim, dim);
		}

		/**
		 * Deadbands the joystick in a circle based on a given radius around the origin
		 * @param radius 0-sqrt(2)
		 */
		public static Function<JoystickFrame, JoystickFrame> makeRadialDeadband(final double radius) {
			return (stick) -> {
				final double scaledRadius = radius / Math.sqrt(2.0); // this is from 0-1, not 0-sqrt(2)
				final boolean valid = radius < stick.getMagnitude();

				return new JoystickFrame(
						valid ? stick.getX() : 0,
						valid ? stick.getY() : 0,
						stick.getZ()
				);
			};
		}

		/**
		 * Deadbands the joystick in the shape of a 2/3 degree superellipse,
		 * otherwise known as a squashed astroid
		 * The logic behind this is that 
		 * 	1. The controller is unlikely to come to a rest offset from the origin
		 * 		in two axis of movement
		 *  2. Movement in more than one axis is unlikely to be accidental. 
		 *  3. Therefore, an astroid deadzone is very efficient as it will protect
		 *  	against slight default percent errors,
		 *  	and will still allow for quick multiaxial movement.
		 * @param limit Approximately the total width of the astroid. 
		 */
		public static Function<JoystickFrame, JoystickFrame> makeAstroidDeadband(final double limit) {
			return (stick) -> {
				final double value = 
						Math.pow(Math.abs(stick.getX()), TWO_THIRDS) 
						+ Math.pow(Math.abs(stick.getY()), TWO_THIRDS);

				final boolean valid = limit < value;

				return new JoystickFrame(
						valid ? stick.getX() : 0,
						valid ? stick.getY() : 0,
						stick.getZ()
				);
			};
		}

		/**
		 * Deadbands the joystick in the shape of a square rotated 45 degrees
		 * @param diagonal The length of the diagonal of the created square
		 */
		public static Function<JoystickFrame, JoystickFrame> makeDiamondDeadband(final double diagonal) {
			return (stick) -> {
				final double x = stick.getX(),
						     y = stick.getY();

				final boolean valid = (diagonal / 2) < (Math.sqrt(Math.abs(x) + Math.abs(y)));

				return new JoystickFrame(
						valid ? stick.getX() : 0,
						valid ? stick.getY() : 0,
						stick.getZ()
				);
			};
		}
		
		/**
		 * Deadband the joystick twist
		 * @param threshold The minimum proportion of rotation to get a value
		 */
		public static Function<JoystickFrame, JoystickFrame> makeZDeadband(final double threshold) {
			return (stick) -> new JoystickFrame(
					stick.getX(),
					stick.getY(),
					deadband(stick.getZ(), threshold)
			);
		}

		/**
		 * Produces a function which inverts the inputs
		 */
		public static Function<JoystickFrame, JoystickFrame> makeInverts(
				final boolean x,
				final boolean y,
				final boolean z
		) {
			return (stick) -> new JoystickFrame(
					stick.getX() * (x ? -1 : 1),
					stick.getY() * (y ? -1 : 1),
					stick.getZ() * (z ? -1 : 1)
			);
		}
	}

	protected final FunctionPipeline<JoystickFrame> m_pipeline;
	protected final String m_name;
	private JoystickFrame m_lastFrame;

	/**
	 * @return The default function pipeline
	 */
	public static FunctionPipeline<JoystickFrame> defaultPipeline() {
		return new FunctionPipeline<>(
				DeadbandingOptions.makeInverts(false, true, false),
				DeadbandingOptions.makeRectangleDeadband(0.12, 0.12),
				DeadbandingOptions.makeZDeadband(0.08)
		);
	}

	/**
	 * The main constructor for the class, all other ones just provide
	 * default values for this one
	 * @param name What to name the stick for logging purposes
	 * @param port Which port to use for the Joystick
	 * @param pipeline The functions and filters to apply to the {@link JoystickFrame}s
	 */
	public Gemstick(
			final String name,
			final int port,
			final FunctionPipeline<JoystickFrame> pipeline
	) {
		super(port);

		m_name = name;
		m_pipeline = pipeline;
		m_lastFrame = new JoystickFrame(this);

		instances++;
	}

	// easy constructor
	public Gemstick(final String name, final int port) {
		this(name, port, defaultPipeline());
	}

	// super easy constructor
	public Gemstick(final int port) {
		this("Jostick " + String.format("%02d", instances), port);
	}

	/**
	 * Gets the POV value of {@link Joystick} but makes it a usable value
	 * @return The POV state with a non-360 degree value3
	 */
	public POVState getPOVState() {
		return POVState.ofDegrees(super.getPOV());
	}

	public HIDType getType() {
		return HIDType.kHIDJoystick;
	}

	public final String getName() {
		return m_name;
	}

	/**
	 * @return Efficient data collection- only polls values if they're past the frame length
	 */
	private boolean isLastFrameExpired() {
		return System.currentTimeMillis() - m_lastFrame.getTime() > FRAME_LENGTH_MS;
	}

	/**
	 * @return The most recent relevant joystick values, and/or pulls new ones
	 */
	public JoystickFrame getRawFrame() {
		if (isLastFrameExpired()) {
			m_lastFrame = new JoystickFrame(this);
		}
		
		return m_lastFrame;
	}

	/**
	 * {@link Gemstick#getRawFrame()} but with the pipeline applied
	 * @return The filtered and modified frame
	 */
	public JoystickFrame getFrame() {
		return m_pipeline.apply(getRawFrame());
	}

	/**
	 * Allows one to apply a {@link Lens} to a {@link JoystickFrame}
	 * to get a specific property
	 * @param lens The lens which will collect the entry from the frame
	 * @return The specified property
	 */
	public double get(final Lens lens) {
		return lens.apply(getFrame());
	}
}
