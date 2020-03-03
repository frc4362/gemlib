package com.gemsrobotics.lib.subsystems;

import com.ctre.phoenix.CANifier;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.telemetry.reporting.ReportingEndpoint;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import org.opencv.ml.EM;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.IntStream;

import static java.lang.Math.min;

public abstract class Limelight extends Subsystem {
    private static final String DEFAULT_NAME = "limelight";

    protected static final double[] EMPTY = new double[]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    protected static final Rotation FOV_HORIZONTAL_HALF = Rotation.degrees(27.0);
    protected static final Rotation FOV_VERTICAL_HALF = Rotation.degrees(20.5);

	protected static final double IMAGE_CAPTURE_LATENCY_S = 11e-3;

    private final NetworkTable m_table;
    private final NetworkTableEntry
            m_existsEntry, m_offsetHorizontalEntry, m_offsetVerticalEntry,
            m_areaEntry, m_skewEntry, m_modeLedEntry, m_modeCameraEntry, m_pipelineEntry,
            m_cornerXEntry, m_cornerYEntry, m_latencyEntry;

    protected final PeriodicIO m_periodicIO;

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
        public double timestamp = Double.NaN;
        public boolean targetPresent = false;
        public Rotation offsetHorizontal = Rotation.identity();
        public Rotation offsetVertical = Rotation.identity();
        public double targetArea = 0;
        public double targetSkew = 0;
        public double[] cornersX;
        public double[] cornersY;

        // OUTPUTS
        public LEDMode ledMode = LEDMode.values()[0];
        public CameraMode cameraMode = CameraMode.values()[0];
        public int pipeline = 0;
    }

    @Override
    protected synchronized final void readPeriodicInputs(final double now) {
        // INPUTS
        m_periodicIO.latency = IMAGE_CAPTURE_LATENCY_S + (m_latencyEntry.getDouble(Double.POSITIVE_INFINITY) * 1e-3);
        m_periodicIO.timestamp = now - m_periodicIO.latency;
        m_periodicIO.targetPresent = m_existsEntry.getDouble(0) == 1.0;
        // invert this so it is CCW-positive
        m_periodicIO.offsetHorizontal = Rotation.degrees(m_offsetHorizontalEntry.getDouble(0)).inverse();
        m_periodicIO.offsetVertical = Rotation.degrees(m_offsetVerticalEntry.getDouble(0));
        m_periodicIO.targetArea = m_areaEntry.getDouble(0.0);
        m_periodicIO.targetSkew = m_skewEntry.getDouble(0.0);

        m_periodicIO.cornersX = m_cornerXEntry.getDoubleArray(EMPTY);
		m_periodicIO.cornersY = m_cornerYEntry.getDoubleArray(EMPTY);

//        final var xs = m_cornerXEntry.getDoubleArray(EMPTY);
//        final var ys = m_cornerYEntry.getDoubleArray(EMPTY);
//        m_periodicIO.corners = makeCornerArrays(xs, ys);

        // OUTPUTS
        m_periodicIO.ledMode = LEDMode.values()[(int) m_pipelineEntry.getDouble(0.0)];
        m_periodicIO.cameraMode = CameraMode.values()[(int) m_modeCameraEntry.getDouble(0.0)];
        m_periodicIO.pipeline = (int) m_pipelineEntry.getDouble(0.0);
    }

    public synchronized final void setLEDMode(final LEDMode mode) {
	    if (mode != m_periodicIO.ledMode) {
            m_modeLedEntry.setDouble(mode.ordinal());
            report("Wanted LED mode changed to " + mode.toString() + ".");
        }
    }

    public synchronized final void setCameraMode(final CameraMode mode) {
	    if (mode != m_periodicIO.cameraMode) {
            m_modeCameraEntry.setDouble(mode.ordinal());
            report("Wanted camera mode swapped to " + mode.toString() + ".");
        }
    }

    public synchronized final void setSelectedPipeline(final int p) {
        if (p < 9 && p > 0 && p != m_periodicIO.pipeline) {
            m_pipelineEntry.setDouble(p);
            report("Wanted pipeline swapped to " + p + ".");
        }
    }

    public final Optional<Double> getRawProperty(final String property) {
        final var val = m_table.getEntry(property).getDouble(Double.NaN);
        final var ret = Optional.ofNullable(Double.isNaN(val) ? null : val);

        if (ret.isEmpty()) {
            report(ReportingEndpoint.Event.Kind.WARNING, "Attempted to access nonexistent property \"" + property + "\".");
        }

        return ret;
    }

    @Override
    public final void setSafeState() {
        // its a limelight, I don't think it can *be* unsafe...
    }

    public synchronized final boolean isAlive() {
	    return m_periodicIO.latency < Double.POSITIVE_INFINITY;
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
		return m_periodicIO.targetPresent;
	}

    /**
     * @return Offset from the center in radians, positive is to the left of view origin
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
	    return Arrays.asList(makeCornerArrays(m_periodicIO.cornersX, m_periodicIO.cornersY));
    }

    protected static Translation[] makeCornerArrays(final double[] xs, final double[] ys) {
        return IntStream.range(0, min(xs.length, ys.length))
                        .mapToObj(i -> new Translation(xs[i], ys[i]))
                        .toArray(Translation[]::new);
    }
}
