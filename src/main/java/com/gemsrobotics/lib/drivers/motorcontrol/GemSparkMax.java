package com.gemsrobotics.lib.drivers.motorcontrol;

import com.gemsrobotics.lib.controls.PIDFController;
import com.gemsrobotics.lib.telemetry.reporting.Reportable;
import com.gemsrobotics.lib.telemetry.reporting.ReportingEndpoint.Event.Kind;
import com.revrobotics.*;

import java.util.function.Supplier;

public final class GemSparkMax extends CANSparkMax implements MotorController, Reportable {
    private static final int MAX_TRIES = 3;

    @Override
    public String getName() {
        return m_name;
    }

    private ControlType m_lastDemandType;
    private double m_lastDemand;
    private double m_lastFeedforward;
    private CANSparkMax m_leader;
    private boolean m_hasMotionProfilingBeenConfigured;
    private int m_selectedProfileID;

    private final String m_name;
    private final CANPIDController m_controller;
    private final CANEncoder m_encoder;

    protected GemSparkMax(final int port) {
		super(port, MotorType.kBrushless);

		m_name = "SparkMAX-" + port;

		runWithRetries(() -> enableVoltageCompensation(12.0));

		m_controller = getPIDController();
		m_encoder = getEncoder();

		m_hasMotionProfilingBeenConfigured = false;
		m_selectedProfileID = 0;

		m_lastDemand = Double.NaN;
		m_lastDemandType = null;
		m_leader = null;
	}

	@Override
	public CANError follow(final CANSparkMax leader, final boolean invert) {
		m_leader = leader;
		return super.follow(leader, invert);
	}

	public CANSparkMax getLeader() {
		return m_leader;
	}

    public void set(final ControlType type, final double demand) {
        set(type, demand, 0.0);
    }

	public void set(final ControlType type, final double demand, final double feedforward) {
		if (demand != m_lastDemand || type != m_lastDemandType) {
			m_lastDemandType = type;
			m_lastDemand = demand;
			m_controller.setReference(demand, type, m_selectedProfileID, feedforward);
		}
	}

    @Override
    public double getVoltageInput() {
        return getBusVoltage();
    }

    @Override
    public double getVoltageOutput() {
        return getAppliedOutput() * getBusVoltage();
    }

    @Override
    public double getDrawnCurrent() {
        return getOutputCurrent();
    }

    @Override
    public int getDeviceID() {
        return super.getDeviceId();
    }

    @Override
    public boolean isEncoderPresent() {
        // Sparks have to have an encoder, otherwise they burn themselves out...
        // if this would ever be false, we have bigger problems
        return true;
    }

    @Override
    public void setSelectedProfile(final int profileID) {
        m_selectedProfileID = profileID;
    }

    @Override
    public synchronized boolean follow(final MotorController leader, final boolean invert) {
        if (leader instanceof CANSparkMax) {
            return runWithRetries(() -> follow((CANSparkMax) leader, invert));
        } else {
            return false;
        }
    }

    @Override
    public synchronized boolean setCurrentLimit(final int currentLimitAmps) {
        return runWithRetries(() -> setSmartCurrentLimit(currentLimitAmps));
    }

    @Override
    public synchronized boolean setInvertedOutput(final boolean inverted) {
        setInverted(inverted);
        return true;
    }

    @Override
    public synchronized boolean setNeutralBehaviour(final NeutralBehaviour mode) {
	    final var vendorMode = mode == NeutralBehaviour.BRAKE ? IdleMode.kBrake : IdleMode.kCoast;
        return runWithRetries(() -> setIdleMode(vendorMode));
    }

    @Override
    public synchronized boolean setRotationsPerMeter(final double rotationsPerMeter) {
        boolean success = true;

        success &= runWithRetries(() -> m_encoder.setPositionConversionFactor(rotationsPerMeter));
        success &= runWithRetries(() -> m_encoder.setVelocityConversionFactor(rotationsPerMeter * 60));

        return success;
    }

    @Override
    public synchronized boolean setEncoderPosition(double position) {
        return runWithRetries(() -> m_encoder.setPosition(position));
    }

    @Override
    public boolean setOpenLoopVoltageRampRate(final double timeToRamp) {
        return runWithRetries(() -> setOpenLoopRampRate(timeToRamp));
    }

    @Override
    public boolean setClosedLoopVoltageRampRate(double timeToRamp) {
        return runWithRetries(() -> setClosedLoopRampRate(timeToRamp));
    }

    @Override
    public double getPositionMotorRotations() {
        return m_encoder.getPosition() / m_encoder.getPositionConversionFactor();
    }

    @Override
    public synchronized boolean setPIDF(final PIDFController.Gains gains) {
        boolean success = true;

        success &= runWithRetries(() -> m_controller.setP(gains.kP, m_selectedProfileID));
        success &= runWithRetries(() -> m_controller.setI(gains.kI, m_selectedProfileID));
        success &= runWithRetries(() -> m_controller.setD(gains.kD, m_selectedProfileID));
        success &= runWithRetries(() -> m_controller.setFF(gains.kFF, m_selectedProfileID));

        return success;
    }

    @Override
    public double getVelocityLinearMetersPerSecond() {
        return m_encoder.getVelocity();
    }

    @Override
    public synchronized boolean setMotionParameters(final MotionParameters vars) {
	    boolean success = true;

        success &= runWithRetries(() -> m_controller.setSmartMotionMaxAccel(vars.acceleration, m_selectedProfileID));
        success &= runWithRetries(() -> m_controller.setSmartMotionMaxVelocity(vars.cruiseVelocity, m_selectedProfileID));
        success &= runWithRetries(() -> m_controller.setSmartMotionAllowedClosedLoopError(vars.allowableError, m_selectedProfileID));

        m_hasMotionProfilingBeenConfigured = success;

        return success;
    }

    @Override
    public void setDutyCycle(final double cycle, final double feedforward) {
        set(ControlType.kDutyCycle, cycle, feedforward);
    }

    @Override
    public void setVoltage(final double voltage, final double feedforward) {
        set(ControlType.kVoltage, voltage, feedforward);
    }

    @Override
    public void setVelocityMetersPerSecond(final double velocity, final double feedforward) {
        set(m_hasMotionProfilingBeenConfigured ? ControlType.kSmartVelocity : ControlType.kVelocity, velocity, feedforward);
    }

    @Override
    public void setVelocityMotorRPM(final double rpm, final double feedforward) {
        setVelocityMetersPerSecond(rpm / m_encoder.getVelocityConversionFactor(), feedforward);
    }

    @Override
    public void setPositionMeters(final double position, final double feedforward) {
        set(m_hasMotionProfilingBeenConfigured ? ControlType.kSmartMotion : ControlType.kPosition, position, feedforward);
    }

    @Override
    public void setPositionRotations(final double rotations, final double feedforward) {
        setPositionMeters(rotations * m_encoder.getPositionConversionFactor(), feedforward);
    }

    @Override
    public void setNeutral() {
       set(ControlType.kDutyCycle, 0.0, 0.0);
    }

    @Override
    public double getPositionMeters() {
        return m_encoder.getPosition();
    }

    @Override
    public double getVelocityMotorRPM() {
        return m_encoder.getVelocity() / m_encoder.getVelocityConversionFactor();
    }

    private synchronized boolean runWithRetries(final Supplier<CANError> call) {
	    boolean success;

	    int tries = 0;

	    do {
	        success = call.get() == CANError.kOK;
        } while (!success && tries++ < MAX_TRIES);

	    if (tries >= MAX_TRIES || !success) {
	        report(Kind.ERROR, "Failed to configure SparkMax on Port " + getDeviceId() + "!!");
	        return false;
        } else {
	        return true;
        }
    }
}
