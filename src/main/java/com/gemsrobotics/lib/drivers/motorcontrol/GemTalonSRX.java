package com.gemsrobotics.lib.drivers.motorcontrol;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.gemsrobotics.lib.controls.PIDFController;
import com.gemsrobotics.lib.data.CachedBoolean;
import com.gemsrobotics.lib.telemetry.reporting.Reportable;
import com.gemsrobotics.lib.telemetry.reporting.ReportingEndpoint.Event.Kind;
import com.gemsrobotics.lib.utils.TalonUtils;

import java.util.Objects;
import java.util.function.Supplier;

public class GemTalonSRX extends TalonSRX implements MotorController, Reportable {
    private static final int
            MAX_TRIES = 3,
            TIMEOUT_MS = 10;
    private static final double
            NATIVE_UNITS_PER_ROTATION = 4096;

    @Override
    public String getName() {
        return m_name;
    }

	private ControlMode m_lastMode;
	private DemandType m_lastDemandType;
	private boolean m_hasMotionProfilingBeenConfigured;
	private double m_lastValue, m_lastDemand;
    private int m_selectedProfileID;
	private double m_metersPerRotation;

    private final String m_name;
    private final CachedBoolean m_isEncoderPresent;

	protected GemTalonSRX(final int port, final boolean isSlave) {
		super(port);

		m_name = "TalonSRX-" + (isSlave ? "Slave-" : "") + port;

		enableVoltageCompensation(true);
		runWithRetries(() -> configVoltageCompSaturation(12.0, TIMEOUT_MS));

		m_metersPerRotation = 1.0;
		m_isEncoderPresent = new CachedBoolean(0.05, () -> TalonUtils.isEncoderPresent(this));
        m_selectedProfileID = 0;
        m_hasMotionProfilingBeenConfigured = false;

		m_lastMode = null;
		m_lastDemandType = null;
		m_lastValue = Double.NaN;
		m_lastDemand = Double.NaN;
	}

    public GemTalonSRX(final int port) {
        this(port, false);
    }

	@Override
	public void set(final ControlMode mode, final double value) {
		if (mode != m_lastMode || value != m_lastValue || !Objects.isNull(m_lastDemandType)) {
			super.set(mode, value);

			m_lastMode = mode;
			m_lastValue = value;

            m_lastDemandType = null;
            m_lastDemand = Double.NaN;
		}
	}

	@Override
    public void set(final ControlMode mode, final double value, final DemandType demandType, final double demand) {
        if (mode != m_lastMode || value != m_lastValue || demandType != m_lastDemandType || demand != m_lastDemand) {
            super.set(mode, value, demandType, demand); // multiply by 1023 because it wants throttle units on the output

            m_lastMode = mode;
            m_lastValue = value;

            m_lastDemandType = demandType;
            m_lastDemand = demand;
        }
    }

    @Override
    public double getVoltageInput() {
	    return getBusVoltage();
    }

    @Override
    public double getVoltageOutput() {
        return getMotorOutputVoltage();
    }

    @Override
    public double getDrawnCurrent() {
        return getOutputCurrent();
    }

    @Override
    public int getDeviceID() {
        return super.getDeviceID();
    }

    @Override
    public boolean isEncoderPresent() {
        return m_isEncoderPresent.get();
    }

    @Override
    public void setSelectedProfile(final int profileID) {
        m_selectedProfileID = profileID;
    }

    @Override
    public synchronized boolean follow(final MotorController other, final boolean invert) {
        if (other instanceof TalonSRX) {
            setInverted(invert ? InvertType.OpposeMaster : InvertType.FollowMaster);
            super.set(ControlMode.Follower, ((TalonSRX) other).getDeviceID());
            return true;
        } else {
            return false;
        }
    }

    @Override
    public synchronized boolean setCurrentLimit(final int currentLimitAmps) {
	    boolean success = true;

	    success &= runWithRetries(() -> {
                enableCurrentLimit(true);
                return getLastError();
        });
        success &= runWithRetries(() -> configContinuousCurrentLimit(currentLimitAmps));
        success &= runWithRetries(() -> configPeakCurrentLimit(0));

        return success;
    }

    public synchronized boolean disableCurrentLimit() {
	    return runWithRetries(() -> {
            enableCurrentLimit(false);
            return getLastError();
        });
    }

    @Override
    public boolean setInvertedOutput(final boolean inverted) {
        setInverted(inverted);
        return true;
    }

    @Override
    public boolean setNeutralBehaviour(final NeutralBehaviour mode) {
	    setNeutralMode(mode == NeutralBehaviour.BRAKE ? NeutralMode.Brake : NeutralMode.Coast);
        return true;
    }

    @Override
    public boolean setRotationsPerMeter(final double rotationsPerMeter) {
        m_metersPerRotation = (1 / rotationsPerMeter);
	    return true;
    }

    @Override
    public synchronized boolean setEncoderPosition(final double position) {
        return runWithRetries(() -> setSelectedSensorPosition((int) position, m_selectedProfileID, TIMEOUT_MS));
    }

    @Override
    public boolean setOpenLoopVoltageRampRate(final double timeToRamp) {
        return runWithRetries(() -> configOpenloopRamp(timeToRamp, TIMEOUT_MS));
    }

    @Override
    public boolean setClosedLoopVoltageRampRate(final double timeToRamp) {
	    return runWithRetries(() -> configClosedloopRamp(timeToRamp, TIMEOUT_MS));
    }

    @Override
    public double getPositionMeters() {
	    return getPositionMotorRotations() * m_metersPerRotation;
    }

    @Override
    public double getPositionMotorRotations() {
        return nativeUnits2Rotations(getSelectedSensorPosition(m_selectedProfileID));
    }

    @Override
    public double getVelocityLinearMetersPerSecond() {
        return (getVelocityMotorRPM() / 60) * m_metersPerRotation;
    }

    @Override
    public double getVelocityMotorRPM() {
        return nativeUnits2RPM(getSelectedSensorVelocity(m_selectedProfileID));
    }

    @Override
    public synchronized boolean setPIDF(final PIDFController.Gains gains) {
	    boolean success = true;

	    success &= runWithRetries(() -> config_kP(m_selectedProfileID, gains.kP, TIMEOUT_MS));
	    success &= runWithRetries(() -> config_kI(m_selectedProfileID, gains.kI, TIMEOUT_MS));
	    success &= runWithRetries(() -> config_kD(m_selectedProfileID, gains.kD, TIMEOUT_MS));
	    success &= runWithRetries(() -> config_kF(m_selectedProfileID, gains.kFF, TIMEOUT_MS));
	    success &= runWithRetries(() -> configAllowableClosedloopError(m_selectedProfileID, (int) gains.tolerance, TIMEOUT_MS));

	    return success;
    }

    @Override
    public boolean setMotionParameters(final MotionParameters vars) {
        boolean success = true;

        success &= runWithRetries(() -> configMotionAcceleration((int) vars.acceleration, TIMEOUT_MS));
        success &= runWithRetries(() -> configMotionCruiseVelocity((int) vars.cruiseVelocity, TIMEOUT_MS));
        success &= runWithRetries(() -> configAllowableClosedloopError(m_selectedProfileID, (int) vars.allowableError, TIMEOUT_MS));

        m_hasMotionProfilingBeenConfigured = success;

        return success;
    }

    @Override
    public void setDutyCycle(final double cycle, final double feedforward) {
        set(ControlMode.PercentOutput, cycle, DemandType.ArbitraryFeedForward, feedforward);
    }

    @Override
    public void setVoltage(final double voltage, final double feedforward) {
        setDutyCycle(voltage / 12.0, feedforward);
    }

    @Override
    public void setVelocityMetersPerSecond(final double velocity, final double feedforward) {
        setVelocityMotorRPM(velocity / m_metersPerRotation * 60, feedforward);
    }

    @Override
    public void setVelocityMotorRPM(final double rpm, final double feedforward) {
        set(ControlMode.Velocity, RPM2NativeUnits(rpm), DemandType.ArbitraryFeedForward, feedforward);
    }

    @Override
    public void setPositionMeters(final double position, final double feedforward) {
        setPositionRotations(position / m_metersPerRotation, feedforward);
    }

    @Override
    public void setPositionRotations(final double rotations, final double feedforward) {
        set(m_hasMotionProfilingBeenConfigured ? ControlMode.MotionMagic : ControlMode.Position, rotations2NativeUnits(rotations), DemandType.ArbitraryFeedForward, feedforward);
    }

    @Override
    public void setNeutral() {
        set(ControlMode.Disabled, 0.0);
    }

    public synchronized boolean setNominalOutputForward(final double percentOut) {
	    return runWithRetries(() -> configNominalOutputForward(percentOut, TIMEOUT_MS));
    }

    public synchronized boolean setNominalOutputReverse(final double percentOut) {
        return runWithRetries(() -> configNominalOutputReverse(percentOut, TIMEOUT_MS));
    }

    public synchronized boolean setPeakOutputForward(final double percentOut) {
	    return runWithRetries(() -> configPeakOutputForward(percentOut, TIMEOUT_MS));
    }

    public synchronized boolean setPeakOutputReverse(final double percentOut) {
        return runWithRetries(() -> configPeakOutputReverse(percentOut, TIMEOUT_MS));
    }

    public synchronized boolean setNominalOutputs(final double forward, final double reverse) {
	    boolean success = true;

	    success &= setNominalOutputForward(forward);
	    success &= setNominalOutputReverse(reverse);

	    return success;
    }

    public synchronized boolean setPeakOutputs(final double forward, final double reverse) {
        boolean success = true;

        success &= setPeakOutputForward(forward);
        success &= setPeakOutputReverse(reverse);

        return success;
    }

    private static double nativeUnits2Rotations(final double nativeUnits) {
	    return nativeUnits / NATIVE_UNITS_PER_ROTATION;
    }

    private static double nativeUnits2RPM(final double nativeUnits) {
	    return nativeUnits / NATIVE_UNITS_PER_ROTATION * 600.0;
    }

    private static int rotations2NativeUnits(final double rotations) {
	    return (int) (rotations * NATIVE_UNITS_PER_ROTATION);
    }

    private static int RPM2NativeUnits(final double rpm) {
	    return (int) (rpm * NATIVE_UNITS_PER_ROTATION / 600.0);
    }

    private synchronized boolean runWithRetries(final Supplier<ErrorCode> call) {
        boolean success;

        int tries = 0;

        do {
            success = call.get() == ErrorCode.OK;
        } while (!success && tries++ < MAX_TRIES);

        if (tries >= MAX_TRIES || !success) {
            report(Kind.ERROR, "Failed to configure TalonSRX on Port " + getDeviceID() + "!!");
            return false;
        } else {
            return true;
        }
    }
}
