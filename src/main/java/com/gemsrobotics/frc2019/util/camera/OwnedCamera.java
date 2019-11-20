package com.gemsrobotics.frc2019.util.camera;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.UsbCamera;

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
public final class OwnedCamera {
	private static int newId = 0;

	private final UsbCamera m_camera;
	private final CvSink m_sink;

	public OwnedCamera(final UsbCamera camera) {
		m_camera = camera;
		m_sink = new CvSink(String.format("sink_%d", newId++));
		m_sink.setSource(m_camera);
		m_sink.setEnabled(true);
	}

	public UsbCamera getCamera() {
		return m_camera;
	}
}
