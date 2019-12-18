package com.gemsrobotics.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.gemsrobotics.frc2018.Ports;
import com.gemsrobotics.lib.drivers.motorcontrol.GemTalonSRX;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.telemetry.reporting.ReportingEndpoint;
import com.gemsrobotics.lib.utils.FastDoubleToString;
import com.gemsrobotics.lib.data.CachedValue;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.utils.MathUtils;
import io.github.oblarg.oblog.Loggable;
import com.gemsrobotics.lib.controls.PIDFController.Gains;
import io.github.oblarg.oblog.annotations.Log;

import java.util.Map;
import java.util.Objects;

import static java.lang.Math.abs;
import static com.gemsrobotics.lib.utils.MathUtils.coerce;

@SuppressWarnings({"unused", "WeakerAccess"})
public final class Elevator extends Subsystem implements Loggable {
    private static final int OVERRUN_THRESHOLD = 100;

    private static Elevator INSTANCE;

	public static Elevator getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Elevator();
		}

		return INSTANCE;
	}

    private static final Gains GAINS = new Gains(0.235,0.0002,2.0, 0.01, 100);
    private static final MotorController.MotionParameters MOTION_CONFIG = new MotorController.MotionParameters(26000, 22000, 200);
	private static final double GRAVITY_FF = 0.07;

	private static final int
		LIFT_HEIGHT_TICKS = 30100,
		ALLOWABLE_CLOSED_LOOP_ERROR = 200;

	private static final MathUtils.Bounds positionBounds =
            new MathUtils.Bounds(Position.BOTTOM.ticks, Position.TOP.ticks);

	private final GemTalonSRX m_master, m_slave;
	private final PeriodicIO m_periodicIO;

	private Mode m_controlMode;

	private Elevator() {
	    setName("Elevator");

		m_master = MotorControllerFactory.createDefaultTalonSRX(Ports.ELEVATOR_PORT_MASTER);
        m_master.setNominalOutputs(0, 0);
        m_master.setPeakOutputs(1.0, -0.8);
        m_master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

        m_master.setPIDF(GAINS);
        m_master.setMotionParameters(MOTION_CONFIG);

        m_master.configForwardSoftLimitEnable(true);
        m_master.configForwardSoftLimitThreshold(Position.TOP.ticks, 10);
        m_master.configReverseSoftLimitEnable(true);
        m_master.configReverseSoftLimitThreshold(Position.BOTTOM.ticks, 10);

        m_slave = MotorControllerFactory.createSlaveTalonSRX(Ports.ELEVATOR_PORT_SLAVE);
		m_slave.follow(m_master, true);

		m_periodicIO = new PeriodicIO();
	}

    public enum Mode {
		DISABLED,
		OPEN_LOOP,
		MOTION_MAGIC
	}

	public enum Position {
		TOP(1.0),
		SCALE(0.674),
		CARRY(0.233),
		STARTING(0.233),
		CLOSE_THRESHOLD(0.2),
		BOTTOM(-0.006);

		public final int ticks;

		Position(final double percent) {
			ticks = (int) percent * LIFT_HEIGHT_TICKS;
		}
	}

	public static class PeriodicIO implements Loggable {
	    // INPUTS
	    @Log(name="Control Mode")
		public String controlMode;
	    @Log(name="Position (rots)")
		public double position;
	    @Log(name="Velocity (RPM)")
		public double velocity;

        // OUTPUTS
        @Log(name="Open Loop Demand (%)")
		public double openLoopDemand;
        @Log(name="Reference (Nu)")
		public double closedLoopReference;
        @Log(name="Error (Nu)")
		public double closedLoopError;

		public boolean isEncoderPresent;
		public double temperature;
		public double outputCurrent;
	}

	@Override
	public void readPeriodicInputs() {
		m_periodicIO.controlMode = m_controlMode.toString();
		m_periodicIO.position = m_master.getPositionMotorRotations();
		m_periodicIO.velocity = m_master.getVelocityMotorRPM();

		m_periodicIO.closedLoopError = m_master.getClosedLoopError(0);

		m_periodicIO.isEncoderPresent = m_master.isEncoderPresent();
		m_periodicIO.temperature = m_master.getTemperature();
		m_periodicIO.outputCurrent = m_master.getOutputCurrent();
	}

	private boolean isMotorInUnsafePosition(final GemTalonSRX device) {
		return positionBounds.isValid(device.getSelectedSensorPosition(0));
	}

	private synchronized void setNeutralMode(final NeutralMode mode) {
		m_master.setNeutralMode(mode);
		m_slave.setNeutralMode(mode);
	}

	private synchronized void setSensorPosition(final int position) {
		m_master.setSelectedSensorPosition(position, 0, 10);
		m_slave.setSelectedSensorPosition(position, 0, 10);
	}

	public synchronized void setDisabled() {
		if (m_controlMode != Mode.DISABLED) {
			m_master.set(com.ctre.phoenix.motorcontrol.ControlMode.Disabled, 0);
			m_slave.set(com.ctre.phoenix.motorcontrol.ControlMode.Disabled, 0);
            m_controlMode = Mode.DISABLED;
		}
	}

	public synchronized void setOpenLoop(final double speed) {
        if (m_controlMode != Mode.OPEN_LOOP) {
            m_controlMode = Mode.OPEN_LOOP;
        }

        m_periodicIO.openLoopDemand = coerce(-1, speed, +1);
	}

	public synchronized void setReference(final Position reference) {
		setReference(reference.ticks);
	}

	private synchronized void setReference(final double newReference) {
        if (m_controlMode != Mode.MOTION_MAGIC) {
            m_controlMode = Mode.MOTION_MAGIC;
            m_periodicIO.openLoopDemand = 0;
        }

        m_periodicIO.closedLoopReference = positionBounds.coerce(newReference);
	}

	public synchronized void adjustReference(final double adjustment) {
		final var newRef = m_periodicIO.closedLoopReference + (adjustment * LIFT_HEIGHT_TICKS);
		setReference(newRef);
	}

	@Override
	public void onCreate(final double timestamp) {
		synchronized (this) {
			setSensorPosition(Position.STARTING.ticks);
			setReference(Position.STARTING);
		}
	}

	@Override
	public synchronized void onEnable(final double timestamp) {
        setNeutralMode(NeutralMode.Brake);
	}

	@Override
	public synchronized void onUpdate(final double timestamp) {
        switch (m_controlMode) {
            case DISABLED:
                break;
            case OPEN_LOOP:
                m_master.setDutyCycle(m_periodicIO.openLoopDemand);
                break;
            case MOTION_MAGIC:
                m_master.setPositionRotations(m_periodicIO.closedLoopReference, GRAVITY_FF);
                break;
            default:
                report(ReportingEndpoint.Event.Kind.ERROR, "Unexpected Elevator Control Mode " + m_controlMode.toString());
                break;
        }
	}

	@Override
	public synchronized void onStop(double timestamp) {
        setSafeState();
	}

	@Override
	public synchronized void setSafeState() {
        setNeutralMode(NeutralMode.Brake);
        setDisabled();
	}

	public synchronized boolean isAtReference(final double epsilon) {
		return abs(m_periodicIO.closedLoopError) < epsilon;
	}

	public synchronized double getReference() {
		return m_periodicIO.closedLoopReference;
	}

	public synchronized double getPosition() {
		return m_periodicIO.position;
	}

	public synchronized Mode getControlMode() {
		return m_controlMode;
	}

	@Override
	public synchronized FaultedResponse checkFaulted() {
		var ret = FaultedResponse.NONE;

		if (m_periodicIO.isEncoderPresent) {
			report(ReportingEndpoint.Event.Kind.HARDWARE_FAULT, "Encoder missing");
			ret = FaultedResponse.DISABLE_SUBSYSTEM;
		}

		if (isMotorInUnsafePosition(m_master) || isMotorInUnsafePosition(m_slave)) {
			report(ReportingEndpoint.Event.Kind.HARDWARE_FAULT, "Motor in unsafe position",
					Map.of("positionMaster", FastDoubleToString.format(m_master.getSelectedSensorPosition(0)),
						   "positionSlave", FastDoubleToString.format(m_slave.getSelectedSensorPosition(0))));
			ret = FaultedResponse.DISABLE_SUBSYSTEM;
		}

		return ret;
	}
}
