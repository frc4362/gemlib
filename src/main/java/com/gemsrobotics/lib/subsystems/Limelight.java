package com.gemsrobotics.lib.subsystems;

import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.telemetry.reporting.Reporter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.IntStream;

import static java.lang.Math.min;

public abstract class Limelight extends Subsystem {
    private static final String DEFAULT_NAME = "limelight";

	private static final double IMAGE_CAPTURE_LATENCY_S = 11e-3;

    private final NetworkTable m_table;
    private final NetworkTableEntry
            m_existsEntry, m_offsetHorizontalEntry, m_offsetVerticalEntry,
            m_areaEntry, m_skewEntry, m_modeLedEntry, m_modeCameraEntry, m_pipelineEntry,
            m_cornerXEntry, m_cornerYEntry, m_latencyEntry;

    protected final PeriodicIO m_periodicIO;

    protected LEDMode m_modeLed;
    protected CameraMode m_modeCamera;
    protected int m_selectedPipeline;

	protected Limelight(final String name) {
	    setName("Limelight" + (DEFAULT_NAME.equals(name) ? "" : "-" + name));

		m_table = NetworkTableInstance.getDefault().getTable(name);

		m_existsEntry = m_table.getEntry("tv");
		m_offsetHorizontalEntry = m_table.getEntry("tx");
		m_offsetVerticalEntry = m_table.getEntry("ty");
		m_areaEntry = m_table.getEntry("ta");
		m_skewEntry = m_table.getEntry("ts");
		m_modeLedEntry = m_table.getEntry("ledMode");
		m_modeCameraEntry = m_table.getEntry("cameraMode");
		m_pipelineEntry = m_table.getEntry("pipeline");

		m_cornerXEntry = m_table.getEntry("tcornx");
		m_cornerYEntry = m_table.getEntry("tcorny");

		m_latencyEntry = m_table.getEntry("tl");

		m_periodicIO = new PeriodicIO();
	}

	protected Limelight() {
	    this(DEFAULT_NAME);
    }

    public enum LEDMode {
        DEFER,
        OFF,
        BLINK,
        ON
    }

    public enum CameraMode {
        COMPUTER_VISION,
        DRIVER
    }

    protected static class PeriodicIO {
        // INPUTS
        public double latency = IMAGE_CAPTURE_LATENCY_S;
        public boolean targetExists = false;
        public Rotation offsetHorizontal = Rotation.identity();
        public Rotation offsetVertical = Rotation.identity();
        public double targetArea = 0;
        public double targetSkew = 0;
        public Translation[] corners = new Translation[0];

        // OUTPUTS
        public LEDMode ledMode = LEDMode.values()[0];
        public CameraMode cameraMode = CameraMode.values()[0];
        public int pipeline = 0;
    }

    @Override
    protected synchronized final void readPeriodicInputs() {
        // INPUTS
        m_periodicIO.latency = IMAGE_CAPTURE_LATENCY_S + (m_latencyEntry.getDouble(Double.POSITIVE_INFINITY) * 1e-3);
        m_periodicIO.targetExists = m_existsEntry.getDouble(0) == 1.0;
        m_periodicIO.offsetHorizontal = Rotation.degrees(m_offsetHorizontalEntry.getDouble(0));
        m_periodicIO.offsetVertical = Rotation.degrees(m_offsetVerticalEntry.getDouble(0));
        m_periodicIO.targetArea = m_areaEntry.getDouble(0.0);
        m_periodicIO.targetSkew = m_skewEntry.getDouble(0.0);

        final var xs = m_cornerXEntry.getDoubleArray(new double[]{ 0.0, 0.0 });
        final var ys = m_cornerYEntry.getDoubleArray(new double[]{ 0.0, 0.0 });
        m_periodicIO.corners = makeCornerArrays(xs, ys);

        // OUTPUTS
        m_periodicIO.ledMode = m_modeLed;
        m_periodicIO.cameraMode = m_modeCamera;
        m_periodicIO.pipeline = m_selectedPipeline;
    }

    public synchronized final void setLEDMode(final LEDMode mode) {
	    if (mode != m_modeLed) {
            m_modeLedEntry.setDouble(mode.ordinal());
            m_modeLed = mode;
            report("LED mode changed to " + mode.toString() + ".");
        }
    }

    public synchronized final void setCameraMode(final CameraMode mode) {
	    if (mode != m_modeCamera) {
            m_modeCameraEntry.setDouble(mode.ordinal());
            m_modeCamera = mode;
            report("Camera mode swapped to " + mode.toString() + ".");
        }
    }

    public synchronized final void setSelectedPipeline(final int p) {
        if (p < 9 && p > 0 && p != m_selectedPipeline) {
            m_pipelineEntry.setDouble(p);
            m_selectedPipeline = p;
            report("Pipeline swapped to " + p + ".");
        }
    }

    public final Optional<Double> getRawProperty(final String property) {
        final var val = m_table.getEntry(property).getDouble(Double.NaN);
        final var ret = Optional.ofNullable(Double.isNaN(val) ? null : val);

        if (ret.isEmpty()) {
            report(Reporter.Event.Kind.WARNING, "Attempted to access nonexistent property \"" + property + "\".");
        }

        return ret;
    }

    @Override
    public void setSafeState() {
        // its a limelight, I don't think it can *be* unsafe...
    }

    /**
     * @return The delay from image capture -> property transmission, in seconds
     */
    public synchronized final double getLatency() {
        return m_periodicIO.latency;
    }

    /**
     * @return Whether or not any contours are found in frame
     */
	public synchronized final boolean isTargetPresent() {
		return m_periodicIO.targetExists;
	}

    /**
     * @return Offset from the center in radians, positive is to the right of view origin
     */
	public synchronized final Rotation getOffsetHorizontal() {
		return m_periodicIO.offsetHorizontal;
	}

    /**
     * @return Offset from the center, axis is
     */
	public synchronized final Rotation getOffsetVertical() {
		return m_periodicIO.offsetVertical;
	}

    /**
     * @return The % of the frame which the target makes up
     */
	public synchronized final double getArea() {
		return m_periodicIO.targetArea;
	}

	public synchronized final double getSkew() {
	    return m_periodicIO.targetSkew;
    }

    /**
     * @return The corner pixel locations of a target- x axis is pixels left and right of the frame, y is pixels up and down
     */
    public synchronized final List<Translation> getCorners() {
	    return Arrays.asList(m_periodicIO.corners);
    }

    private static Translation[] makeCornerArrays(final double[] xs, final double[] ys) {
        return IntStream.range(0, min(xs.length, ys.length)).mapToObj(i -> new Translation(xs[i], ys[i])).toArray(Translation[]::new);
    }
}
