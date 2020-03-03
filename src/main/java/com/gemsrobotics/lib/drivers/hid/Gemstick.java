package com.gemsrobotics.lib.drivers.hid;

import java.util.function.Function;

import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.data.CachedValue;

import edu.wpi.first.wpilibj.Joystick;

import static java.lang.Math.*;

/**
 * Over-engineered {@link Joystick} extension which has a LOT of behavior
 * Allows for POV controls, layered, sophisticated deadbanding on every axis,
 * and doesn't update values unless it needs to do so
 *
 * @author Ethan Malzone ejmalzone@gmail.com
 */
@SuppressWarnings({"unused", "WeakerAccess"})
public final class Gemstick extends Joystick {
	/**
	 * Effectively a {@link Translation} with a rotation
	 */
	public static class Frame extends Translation {
		private final double z;

		public Frame(final double x, final double y, final double z) {
			super(x, y);
			this.z = z;
		}

		/**
		 * @param stick The joystick from which you want to pull values
		 */
		public Frame(final Joystick stick) {
			this(stick.getX(), stick.getY(), stick.getZ());
		}

		public double z() {
			return z;
		}
	}

    /**
	 * Provides sophisticated deadbanding utilities and options
	 * This is doable as it doesn't matter which order you apply these in
	 */
	@SuppressWarnings("unused")
	public static class Deadbands {
		/**
		 * @return The default function pipeline
		 */
		public static Function<Frame, Frame> defaultPipeline() {
			return Deadbands.makeInverts(true, true, false)
                    .andThen(Deadbands.makeRadialDeadband(0.06))
                    .andThen(Deadbands.makeZDeadband(0.06));
		}

		/**
		 * Scales a value that's from a given range to 1.0 in either direction,
		 * instead of from 0.0-1.0
		 * ie scale(0.75, 0.5) -> 0.5 motor output
		 * [1.0 - range, 1.0] -> [0.0, 1.0]
		 * @param input The number to be scaled
		 * @param range The amount by which the scale should be adjusted
		 * @return The appropriately scaled input
		 */
		private static double scale(final double input, final double range) {
			return (1.0 / range) * (input - ((1.0 - range) * signum(input)));
		}

		// fundamental deadbanding method
		private static double deadband(final double input, final double threshold) {
			return abs(input) > threshold ? scale(input, 1 - threshold) : 0;
		}

		/**
		 * Deadbands the {@link Gemstick} in a rectangle shape
		 * @param limitX The width of the rectangle
		 * @param limitY The height of the rectangle
		 * @return The modified pipeline which will provide this
		 */
		public static Function<Frame, Frame> makeRectangleDeadband(
				final double limitX, final double limitY
		) {
			return (stick) -> {
				final double x = stick.x(),
						     y = stick.y();

				return new Frame(
						abs(x) > limitX ? scale(x, 1 - limitX) : 0,
						abs(y) > limitY ? scale(y, 1 - limitY) : 0,
						stick.z());
			};
		}

		/**
		 * Makes a square-shaped deadband
		 * @param dim The width and height of the square
		 * @return The function which will deadband values in a square
		 * @see Deadbands#makeRectangleDeadband(double, double)
		 */
		public static Function<Frame, Frame> makeSquareDeadband(final double dim) {
			return makeRectangleDeadband(dim, dim);
		}

		/**
		 * Deadbands the joystick in a circle based on a given radius around the origin
		 * @param radius 0-sqrt(2)
		 */
		public static Function<Frame, Frame> makeRadialDeadband(final double radius) {
			return (stick) -> {
				final double scaledRadius = radius / sqrt(2.0); // this is from 0-1, not 0-sqrt(2)
				final boolean valid = radius < stick.norm();

				return new Frame(
						valid ? stick.x() : 0,
						valid ? stick.y() : 0,
						stick.z()
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
		public static Function<Frame, Frame> makeAstroidDeadband(final double limit) {
			return (stick) -> {
				final double value = pow(abs(stick.x()), 2.0 / 3.0) + pow(abs(stick.y()), 2.0 / 3.0);
				final boolean valid = limit < value;

				return new Frame(
						valid ? stick.x() : 0,
						valid ? stick.y() : 0,
						stick.z()
				);
			};
		}

		/**
		 * Deadbands the joystick in the shape of a square rotated 45 degrees
		 * @param diagonal The length of the diagonal of the created square
		 */
		public static Function<Frame, Frame> makeDiamondDeadband(final double diagonal) {
			return (stick) -> {
				final double x = stick.x(),
						     y = stick.y();

				final boolean valid = (diagonal / 2) < (sqrt(Math.abs(x) + Math.abs(y)));

				return new Frame(
						valid ? stick.x() : 0,
						valid ? stick.y() : 0,
						stick.z()
				);
			};
		}
		
		/**
		 * Deadband the joystick twist
		 * @param threshold The minimum proportion of rotation to get a value
		 */
		public static Function<Frame, Frame> makeZDeadband(final double threshold) {
			return (stick) -> new Frame(
					stick.x(),
					stick.y(),
					deadband(stick.z(), threshold)
			);
		}

		/**
		 * Produces a function which inverts the inputs
		 */
		public static Function<Frame, Frame> makeInverts(
				final boolean x,
				final boolean y,
				final boolean z
		) {
			return (stick) -> new Frame(
					stick.x() * (x ? -1 : 1),
					stick.y() * (y ? -1 : 1),
					stick.z() * (z ? -1 : 1)
			);
		}
	}

	private final CachedValue<Frame> m_joystickFrame;

	/**
	 * The main constructor for the class, all other ones just provide
	 * default values for this one
	 * @param port Which port to use for the Joystick
	 * @param pipeline The functions and filters to apply to the {@link Frame}s
	 */
	public Gemstick(final int port, final Function<Frame, Frame> pipeline) {
		super(port);

		m_joystickFrame = new CachedValue<>(
				Frame.class,
				0.02,
				() -> new Frame(this)).map(Frame.class, pipeline);
	}

	// easy constructor
	public Gemstick(final int port) {
		this(port, Deadbands.defaultPipeline());
	}

	/**
	 * Gets the POV value of {@link Joystick} but makes it a usable value
	 * @return The POV getState with a non-360 degree value3
	 */
	public POVDirection getPOVDirection() {
		return POVDirection.ofDegrees(super.getPOV());
	}

	@Override
	public HIDType getType() {
		return HIDType.kHIDJoystick;
	}

	public Frame getFrame() {
		return m_joystickFrame.get();
	}

	public double x() {
		return m_joystickFrame.get().x();
	}

	public double y() {
		return m_joystickFrame.get().y();
	}

	public double z() {
		return m_joystickFrame.get().z();
	}

	public double angle() {
		return m_joystickFrame.get().direction().getDegrees();
	}

	public double norm() {
		return m_joystickFrame.get().norm();
	}
}
