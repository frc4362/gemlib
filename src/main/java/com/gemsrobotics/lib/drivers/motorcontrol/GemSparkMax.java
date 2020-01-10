package com.gemsrobotics.lib.drivers.motorcontrol;

import com.gemsrobotics.lib.controls.PIDFController;
import com.gemsrobotics.lib.telemetry.reporting.Reportable;
import com.gemsrobotics.lib.telemetry.reporting.ReportingEndpoint.Event.Kind;
import com.revrobotics.*;

import java.util.function.Supplier;

import static com.gemsrobotics.lib.utils.MathUtils.Tau;

public final class GemSparkMax implements MotorController<CANSparkMax>, Reportable {
    private static final int MAX_TRIES = 3;

    @Override
    public String getName() {
        return m_name;
    }

    private final String m_name;
    private final CANSparkMax m_internal;
    private final CANPIDController m_controller;
    private final CANEncoder m_encoder;

    private MotorController<CANSparkMax> m_leader;
    private ControlType m_lastDemandType;
    private int m_selectedProfileID;
    private double m_lastDemand, m_lastFeedforward;

    private double m_cylinderToEncoderReduction, m_cylinderRadiusMeters;
    private boolean m_hasMotionProfilingBeenConfigured;

    protected GemSparkMax(final CANSparkMax spark) {
		m_internal = spark;

		m_name = "SparkMAX-" + m_internal.getDeviceId();

		runWithRetries(() -> m_internal.enableVoltageCompensation(12.0));

		m_controller = m_internal.getPIDController();
		m_encoder = m_internal.getEncoder();

		m_hasMotionProfilingBeenConfigured = false;
		m_selectedProfileID = 0;

		m_lastDemand = Double.NaN;
		m_lastDemandType = null;
		m_leader = null;
	}

	@Override
    public MotorController<CANSparkMax> getLeader() {
		return m_leader;
	}

	private void set(final ControlType type, final double demand, final double feedforward) {
		if (type != m_lastDemandType || demand != m_lastDemand || feedforward != m_lastFeedforward) {
			m_lastDemandType = type;
			m_lastDemand = demand;
			m_lastFeedforward = feedforward;
			m_controller.setReference(demand, type, m_selectedProfileID, feedforward);
		}
	}

    @Override
    public boolean setGearingParameters(final GearingParameters gearingParameters) {
        m_cylinderToEncoderReduction = gearingParameters.cylinderToEncoderReduction;
        m_cylinderRadiusMeters = gearingParameters.cylinderRadiusMeters;
        return true;
    }

    @Override
    public CANSparkMax getInternalController() {
        return m_internal;
    }

    @Override
    public double getVoltageInput() {
        return m_internal.getBusVoltage();
    }

    @Override
    public double getVoltageOutput() {
        return m_internal.getAppliedOutput() * m_internal.getBusVoltage();
    }

    @Override
    public double getDrawnCurrent() {
        return m_internal.getOutputCurrent();
    }

    @Override
    public int getDeviceID() {
        return m_internal.getDeviceId();
    }

    @Override
    public boolean isEncoderPresent() {
        // brushless motors attached to Sparks have to have an encoder, otherwise the motor will burn itself out...
        // if this would ever be false, we have bigger problems
        return true;
    }

    @Override
    public void setSelectedProfile(final int profileID) {
        m_selectedProfileID = profileID;
    }

    @Override
    public synchronized boolean follow(final MotorController<CANSparkMax> leader, final boolean invert) {
        return runWithRetries(() -> m_internal.follow(leader.getInternalController(), invert));
    }

    @Override
    public synchronized boolean setCurrentLimit(final int currentLimitAmps) {
        return runWithRetries(() -> m_internal.setSmartCurrentLimit(currentLimitAmps));
    }

    @Override
    public synchronized boolean setInvertedOutput(final boolean inverted) {
        m_internal.setInverted(inverted);
        return true;
    }

    @Override
    public synchronized boolean setNeutralBehaviour(final NeutralBehaviour mode) {
	    final var vendorMode = mode == NeutralBehaviour.BRAKE ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast;
        return runWithRetries(() -> m_internal.setIdleMode(vendorMode));
    }

    @Override
    public synchronized boolean setEncoderRotations(double position) {
        return runWithRetries(() -> m_encoder.setPosition(position));
    }

    @Override
    public boolean setOpenLoopVoltageRampRate(final double timeToRamp) {
        return runWithRetries(() -> m_internal.setOpenLoopRampRate(timeToRamp));
    }

    @Override
    public boolean setClosedLoopVoltageRampRate(double timeToRamp) {
        return runWithRetries(() -> m_internal.setClosedLoopRampRate(timeToRamp));
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
        return getVelocityAngularRadiansPerSecond() * m_cylinderRadiusMeters;
    }

    @Override
    public double getVelocityAngularRPM() {
        return m_encoder.getVelocity() * m_cylinderToEncoderReduction;
    }

    @Override
    public synchronized boolean setMotionParameters(final MotionParameters vars) {
	    boolean success = true;

        success &= runWithRetries(() -> m_controller.setSmartMotionMaxAccel(vars.acceleration, m_selectedProfileID));
        success &= runWithRetries(() -> m_controller.setSmartMotionMaxVelocity(vars.cruiseVelocity, m_selectedProfileID));
        success &= runWithRetries(() -> m_controller.setSmartMotionAllowedClosedLoopError(vars.tolerance, m_selectedProfileID));

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
        setVelocityRPM(velocity / m_cylinderRadiusMeters * 60, feedforward);
    }

    @Override
    public void setVelocityRPM(final double rpm, final double feedforward) {
        set(m_hasMotionProfilingBeenConfigured ? ControlType.kSmartVelocity : ControlType.kVelocity, rpm / m_cylinderToEncoderReduction, feedforward);
    }

    @Override
    public void setPositionMeters(final double meters, final double feedforward) {
        setPositionRotations(meters / (m_cylinderRadiusMeters * Tau));
    }

    @Override
    public void setPositionRotations(final double rotations, final double feedforward) {
        set(m_hasMotionProfilingBeenConfigured ? ControlType.kSmartMotion : ControlType.kPosition, rotations / m_cylinderToEncoderReduction, feedforward);
    }

    @Override
    public void setNeutral() {
       set(ControlType.kDutyCycle, 0.0, 0.0);
    }

    @Override
    public double getPositionMeters() {
        return getPositionRotations() * Tau * m_cylinderRadiusMeters;
    }

    @Override
    public double getPositionRotations() {
        return m_encoder.getPosition() * m_cylinderToEncoderReduction;
    }

    private synchronized boolean runWithRetries(final Supplier<CANError> call) {
	    boolean success;

	    int tries = 0;

	    do {
	        success = call.get() == CANError.kOk;
        } while (!success && tries++ < MAX_TRIES);

	    if (tries >= MAX_TRIES || !success) {
	        report(Kind.ERROR, "Failed to configure SparkMax on Port " + m_internal.getDeviceId() + "!!");
	        return false;
        } else {
	        return true;
        }
    }
}
