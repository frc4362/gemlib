package com.gemsrobotics.lib.subsystems;

import com.gemsrobotics.frc2020.subsystems.TargetServer;
import com.gemsrobotics.lib.math.ViewportCoordinate;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.telemetry.reporting.ReportingEndpoint.Event.Kind;
import com.gemsrobotics.lib.utils.MathUtils;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import java.util.stream.Stream;

import static com.gemsrobotics.lib.utils.MathUtils.epsilonEquals;
import static java.lang.Math.min;
import static java.lang.Math.tan;

public abstract class Limelight extends Subsystem {
    private static final String DEFAULT_NAME = "limelight";

    protected static final double[] NULL_CORNERS = new double[]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

	protected static final Rotation FOV_HORIZONTAL = Rotation.degrees(54.0);
    protected static final Rotation FOV_VERTICAL = Rotation.degrees(41.0);

    protected static final double VIEWPORT_WIDTH = 2.0 * Math.tan(FOV_HORIZONTAL.getRadians() / 2.0);
    protected static final double VIEWPORT_HEIGHT = 2.0 * Math.tan(FOV_VERTICAL.getRadians() / 2.0);

	private static final Comparator<Translation> X_SORT = Comparator.comparingDouble(Translation::x);
	private static final Comparator<Translation> Y_SORT = Comparator.comparingDouble(Translation::y);

    private final NetworkTable m_table;
    private final NetworkTableEntry
            m_existsEntry, m_offsetHorizontalEntry, m_offsetVerticalEntry,
            m_areaEntry, m_skewEntry, m_modeLedEntry, m_modeCameraEntry, m_pipelineEntry,
            m_cornerXEntry, m_cornerYEntry, m_latencyEntry;

    protected final Resolution m_resolution;
    protected final Rotation m_pitch;
    protected final double m_heightLens, m_heightTarget;

    protected final PeriodicIO m_periodicIO;

	protected Limelight(final String name, final Resolution resolution, final Rotation pitch, final double lensHeight, final double targetHeight) {
	    setName("Limelight" + (DEFAULT_NAME.equals(name) ? "" : "-" + name));

		m_resolution = resolution;
		m_pitch = pitch;
		m_heightLens = lensHeight;
		m_heightTarget = targetHeight;

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

	protected Limelight(final Resolution resolution, final Rotation pitch, final double lensHeight, final double targetHeight) {
	    this(DEFAULT_NAME, resolution, pitch, lensHeight, targetHeight);
    }

    public enum Resolution {
		LOW_DEF(90, 320, 240), HIGH_DEF(22, 960, 720);

		private final double captureTimeSeconds;
		private final int widthX, widthY;

		Resolution(final int fps, final int x, final int y) {
			captureTimeSeconds = 1.0 / fps;
			widthX = x;
			widthY = y;
		}
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

    protected class PeriodicIO {
        // INPUTS
        public double latency = m_resolution.captureTimeSeconds;
        public double timestamp = Double.NaN;
        public boolean targetPresent = false;
        public Rotation offsetHorizontal = Rotation.identity();
        public Rotation offsetVertical = Rotation.identity();
        public double targetArea = 0;
        public double targetSkew = 0;
        public Optional<double[]> cornersX;
        public Optional<double[]> cornersY;

        // OUTPUTS
        public LEDMode ledMode = LEDMode.values()[0];
        public CameraMode cameraMode = CameraMode.values()[0];
        public int pipeline = 0;
    }

    @Override
    protected synchronized final void readPeriodicInputs(final double now) {
        // INPUTS
		final double tl = (m_latencyEntry.getDouble(Double.POSITIVE_INFINITY) * 1e-3);

        m_periodicIO.latency = m_resolution.captureTimeSeconds + tl;
        m_periodicIO.timestamp = now - m_periodicIO.latency;

        // has an unsolvable failure mode where deferred LEDs that are turned off may still read false positives
        if (tl != Double.POSITIVE_INFINITY
			&& m_periodicIO.cameraMode == CameraMode.COMPUTER_VISION
			&& (m_periodicIO.ledMode == LEDMode.ON || m_periodicIO.ledMode == LEDMode.DEFER)
		) {
			m_periodicIO.targetPresent = m_existsEntry.getDouble(0) == 1.0;
			// invert this so it is CCW-positive
			m_periodicIO.offsetHorizontal = Rotation.degrees(m_offsetHorizontalEntry.getDouble(0)).inverse();
			m_periodicIO.offsetVertical = Rotation.degrees(m_offsetVerticalEntry.getDouble(0));
			m_periodicIO.targetArea = m_areaEntry.getDouble(0.0);
			m_periodicIO.targetSkew = m_skewEntry.getDouble(0.0);

			final var newXs = m_cornerXEntry.getDoubleArray(NULL_CORNERS);
			final var newYs = m_cornerYEntry.getDoubleArray(NULL_CORNERS);

			m_periodicIO.cornersX = Arrays.equals(newXs, NULL_CORNERS) ? Optional.empty() : Optional.of(newXs);
			m_periodicIO.cornersY = Arrays.equals(newYs, NULL_CORNERS) ? Optional.empty() : Optional.of(newYs);
		} else {
        	m_periodicIO.targetPresent = false;

			m_periodicIO.offsetHorizontal = Rotation.identity();
			m_periodicIO.offsetVertical = Rotation.identity();
			m_periodicIO.targetArea = 0.0;
			m_periodicIO.targetSkew = 0.0;

			m_periodicIO.cornersX = Optional.empty();
			m_periodicIO.cornersY = Optional.empty();
		}

        // OUTPUTS
        m_periodicIO.ledMode = LEDMode.values()[(int) m_modeLedEntry.getDouble(0.0)];
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
            report(Kind.WARNING, "Attempted to access nonexistent property \"" + property + "\".");
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
     * @return The delay from image capture -> property transmission, in seconds, including frame time
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

	public synchronized Optional<TargetServer.GoalState> getCameraToTarget() {
		return getTarget().flatMap(vp -> {
			final var projection = new Translation(vp.getX(), vp.getZ()).rotateBy(m_pitch);

			final double x = projection.x();
			final double y = vp.getY();
			final double z = projection.y();

			final double diff = m_heightTarget - m_heightLens;

			if (z < 0.0 == diff > 0.0) {
				final double scaling = diff / -z;
				final double distance = Math.hypot(x, y) * scaling;
				final var angle = new Rotation(x, y, true);
				final var cameraToTarget = RigidTransform.fromTranslation(Translation.fromPolar(angle, distance));
				return Optional.of(new TargetServer.GoalState(m_periodicIO.timestamp, cameraToTarget));
			} else {
				return Optional.empty();
			}
		});
	}

    protected synchronized final Optional<ViewportCoordinate> getTarget() {
		return getTopCorners().map(corners -> {
			final double slope;

			// if it's not a vertical line
			if (!epsilonEquals(corners.get(1).x(), corners.get(0).x())) {
				slope = (corners.get(1).y() - corners.get(0).y()) / (corners.get(1).x() - corners.get(0).x());
			} else {
				slope = 1.0;
			}

			final var targetPixels = MathUtils.average(corners);

			// put in frame of reference of the robot
			final double nY = -((targetPixels.x() - (m_resolution.widthX / 2.0)) / (m_resolution.widthX / 2.0));
			final double nZ = -((targetPixels.y() - (m_resolution.widthY / 2.0)) / (m_resolution.widthY / 2.0));

			final double y = VIEWPORT_WIDTH / 2 * nY;
			final double z = VIEWPORT_HEIGHT / 2 * nZ;

			return new ViewportCoordinate(y, z, slope);
		});
	}

    /**
     * @return The corner pixel locations of a target- x axis is pixels left and right of the frame, y is pixels up and down
     */
    public synchronized final Optional<List<Translation>> getRawCorners() {
    	return m_periodicIO.cornersX.flatMap(xs -> m_periodicIO.cornersY.map(ys ->
			 IntStream.range(0, min(xs.length, ys.length))
			   .mapToObj(i -> new Translation(xs[i], ys[i]))
			   .collect(Collectors.toList())));
    }

    private synchronized Optional<List<Translation>> getTopCorners() {
    	return getRawCorners().flatMap(corners -> {
    		if (corners.size() != 8) {
    			return Optional.empty();
			}

			corners.sort(X_SORT);

			final var left = corners.subList(0, 4);
			final var right = corners.subList(4, 8);

			left.sort(Y_SORT);
			right.sort(Y_SORT);

			final var leftTop = left.subList(0, 2);
			final var rightTop = right.subList(0, 2);

			leftTop.sort(X_SORT);
			rightTop.sort(X_SORT);

			return Optional.of(List.of(leftTop.get(0), rightTop.get(0)));
		});
	}
}
