package com.gemsrobotics.lib.drivers;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;

/**
 * A class which retains a pointer to an instance of {@link UsbCamera}
 * so that it cannot pass out of our ownership.
 * Notably, this makes it easier to swap camera views during runtime
 * without a moment of lag and dropped frames, as the program no longer
 * has to reacquire ownership of the camera.
 *
 * @author Ethan Malzone FRC4362
 * reach me at ejmalzone@gmail.com
 * or Ethab#4362 on Discord
 */
@SuppressWarnings("FieldCanBeLocal")
public final class OwnedUsbCamera {
	private static int newId = 0;

	private final UsbCamera m_camera;
	private final CvSink m_sink;

	public OwnedUsbCamera(final UsbCamera camera) {
		m_camera = camera;
		m_sink = new CvSink("sink_" + newId++);
		m_sink.setSource(m_camera);
		m_sink.setEnabled(true);

		System.out.println("Ensuring ownership of UsbCamera at " + camera.getPath() + ".");
	}

	public UsbCamera getCamera() {
		return m_camera;
	}
}
