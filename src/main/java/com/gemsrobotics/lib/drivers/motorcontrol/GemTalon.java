package com.gemsrobotics.lib.drivers.motorcontrol;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.gemsrobotics.lib.controls.PIDFController;
import com.gemsrobotics.lib.data.CachedBoolean;
import com.gemsrobotics.lib.telemetry.reporting.Reportable;
import com.gemsrobotics.lib.telemetry.reporting.ReportingEndpoint;
import com.gemsrobotics.lib.utils.TalonUtils;

import java.util.function.Supplier;

import static com.gemsrobotics.lib.utils.MathUtils.Tau;

public class GemTalon<TalonType extends BaseTalon> implements MotorController<TalonType>, Reportable {
	private static final int
			MAX_TRIES = 3,
			TIMEOUT_MS = 10;

	@Override
	public String getName() {
		return m_name;
	}

	private final String m_name;
	private final TalonType m_internal;
	private final CachedBoolean m_isEncoderPresent;
	private final double m_ticksPerRotation;

	private ControlMode m_lastMode;
	private DemandType m_lastDemandType;
	private int m_selectedProfileID;
	private double m_lastValue, m_lastDemand;

	private double m_cylinderToEncoderReduction, m_cylinderRadiusMeters;
	private boolean m_isMotionProfilingConfigured;
	private MotorController<TalonType> m_leader;

	protected GemTalon(final TalonType talon, final boolean isSlave) {
		m_internal = talon;

		final boolean isFX = talon instanceof TalonFX;

		m_name = "Talon" + (isFX ? "FX" : "SRX") + "-" + (isSlave ? "Slave-" : "") + talon.getDeviceID();

		m_internal.enableVoltageCompensation(true);
		runWithRetries(() -> m_internal.configVoltageCompSaturation(12.0, TIMEOUT_MS));

		if (isFX) {
			m_isEncoderPresent = new CachedBoolean(Double.POSITIVE_INFINITY, () -> true);
			m_ticksPerRotation = 2048;
		} else {
			m_isEncoderPresent = new CachedBoolean(0.05, () -> TalonUtils.isEncoderPresent((TalonSRX) m_internal));
			m_ticksPerRotation = 4096;
		}

		m_selectedProfileID = 0;
		m_isMotionProfilingConfigured = false;

		m_lastMode = null;
		m_lastDemandType = null;
		m_lastValue = Double.NaN;
		m_lastDemand = Double.NaN;
	}

	public GemTalon(final TalonType talon) {
		this(talon, false);
	}

	@Override
	public TalonType getInternalController() {
		return m_internal;
	}

	@Override
	public double getVoltageInput() {
		return m_internal.getBusVoltage();
	}

	@Override
	public double getVoltageOutput() {
		return m_internal.getMotorOutputVoltage();
	}

	@Override
	public double getDrawnCurrent() {
		return m_internal.getStatorCurrent();
	}

	@Override
	public int getDeviceID() {
		return m_internal.getDeviceID();
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
	public MotorController<TalonType> getLeader() {
		return m_leader;
	}

	@Override
	public synchronized boolean follow(final MotorController<TalonType> other, final boolean invert) {
		m_internal.setInverted(invert ? InvertType.OpposeMaster : InvertType.FollowMaster);
		m_internal.set(ControlMode.Follower, other.getDeviceID());
		m_leader = other;
		return true;
	}

	private void set(final ControlMode mode, final double value, final DemandType demandType, final double demand) {
		if (mode != m_lastMode || value != m_lastValue || demandType != m_lastDemandType || demand != m_lastDemand) {
			m_internal.set(mode, value, demandType, demand);

			m_lastMode = mode;
			m_lastValue = value;

			m_lastDemandType = demandType;
			m_lastDemand = demand;
		}
	}

	@Override
	public synchronized boolean setCurrentLimit(final int currentLimitAmps) {
//		boolean success = true;
//
//		success &= runWithRetries(() -> {
//			m_internal.enableCurrentLimit(true);
//			return m_internal.getLastError();
//		});
//		success &= runWithRetries(() -> m_internal.configContinuousCurrentLimit(currentLimitAmps));
//		success &= runWithRetries(() -> m_internal.configPeakCurrentLimit(0));
//
//		return success;

		return false;
	}

	public synchronized boolean disableCurrentLimit() {
//		return runWithRetries(() -> {
//			m_internal.enableCurrentLimit(false);
//			return m_internal.getLastError();
//		});

		return true;
	}

	@Override
	public boolean setInvertedOutput(final boolean inverted) {
		m_internal.setInverted(inverted);
		return true;
	}

	@Override
	public boolean setNeutralBehaviour(final NeutralBehaviour mode) {
		m_internal.setNeutralMode(mode == NeutralBehaviour.BRAKE ? NeutralMode.Brake : NeutralMode.Coast);
		return true;
	}

	@Override
	public boolean setGearingParameters(final GearingParameters gearingParameters) {
		m_cylinderToEncoderReduction = gearingParameters.cylinderToEncoderReduction;
		m_cylinderRadiusMeters = gearingParameters.cylinderRadiusMeters;
		return true;
	}

	@Override
	public synchronized boolean setEncoderRotations(final double position) {
		return runWithRetries(() -> m_internal.setSelectedSensorPosition((int) position, m_selectedProfileID, TIMEOUT_MS));
	}

	@Override
	public synchronized boolean setOpenLoopVoltageRampRate(final double timeToRamp) {
		return runWithRetries(() -> m_internal.configOpenloopRamp(timeToRamp, TIMEOUT_MS));
	}

	@Override
	public synchronized boolean setClosedLoopVoltageRampRate(final double timeToRamp) {
		return runWithRetries(() -> m_internal.configClosedloopRamp(timeToRamp, TIMEOUT_MS));
	}

	@Override
	public void setNeutral() {
		set(ControlMode.Disabled, 0.0, DemandType.ArbitraryFeedForward, 0.0);
	}

	@Override
	public double getPositionMeters() {
		return getPositionRotations() * m_cylinderRadiusMeters;
	}

	@Override
	public double getPositionRotations() {
		return nativeUnits2Rotations(m_internal.getSelectedSensorPosition(m_selectedProfileID));
	}

	@Override
	public synchronized double getVelocityLinearMetersPerSecond() {
		return (getVelocityAngularRPM() / 60) * m_cylinderRadiusMeters;
	}

	@Override
	public synchronized double getVelocityAngularRPM() {
		return nativeUnits2RPM(m_internal.getSelectedSensorVelocity(m_selectedProfileID));
	}

	@Override
	public synchronized boolean setPIDF(final PIDFController.Gains gains) {
		boolean success = true;

		success &= runWithRetries(() -> m_internal.config_kP(m_selectedProfileID, gains.kP, TIMEOUT_MS));
		success &= runWithRetries(() -> m_internal.config_kI(m_selectedProfileID, gains.kI, TIMEOUT_MS));
		success &= runWithRetries(() -> m_internal.config_kD(m_selectedProfileID, gains.kD, TIMEOUT_MS));
		success &= runWithRetries(() -> m_internal.config_kF(m_selectedProfileID, gains.kFF, TIMEOUT_MS));
		success &= runWithRetries(() -> m_internal.configAllowableClosedloopError(m_selectedProfileID, (int) gains.tolerance, TIMEOUT_MS));

		return success;
	}

	@Override
	public synchronized boolean setMotionParameters(final MotionParameters vars) {
		final var cruiseVelocityNativeUnits = RPM2NativeUnitsPer100ms(vars.cruiseVelocity / (Tau * m_cylinderRadiusMeters) * 60);
		final var accelerationNativeUnits = RPM2NativeUnitsPer100ms(vars.acceleration / (Tau * m_cylinderRadiusMeters) * 60);
		final var toleranceNativeUnits = rotations2NativeUnits(vars.tolerance / (Tau * m_cylinderRadiusMeters));

		boolean success = true;

		success &= runWithRetries(() -> m_internal.configMotionAcceleration(cruiseVelocityNativeUnits, TIMEOUT_MS));
		success &= runWithRetries(() -> m_internal.configMotionCruiseVelocity(accelerationNativeUnits, TIMEOUT_MS));
		success &= runWithRetries(() -> m_internal.configAllowableClosedloopError(m_selectedProfileID, toleranceNativeUnits, TIMEOUT_MS));

		m_isMotionProfilingConfigured = success;

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
		setVelocityRPM(velocity / m_cylinderRadiusMeters * 60, feedforward);
	}

	@Override
	public void setVelocityRPM(final double rpm, final double feedforward) {
		set(ControlMode.Velocity, RPM2NativeUnitsPer100ms(rpm), DemandType.ArbitraryFeedForward, feedforward);
	}

	@Override
	public void setPositionMeters(final double meters, final double feedforward) {
		setPositionRotations(meters / (Tau * m_cylinderRadiusMeters), feedforward);
	}

	@Override
	public synchronized void setPositionRotations(final double rotations, final double feedforward) {
		set(m_isMotionProfilingConfigured ? ControlMode.MotionMagic : ControlMode.Position, rotations2NativeUnits(rotations / m_cylinderToEncoderReduction), DemandType.ArbitraryFeedForward, feedforward);
	}

	public boolean setNominalOutputForward(final double percentOut) {
		return runWithRetries(() -> m_internal.configNominalOutputForward(percentOut, TIMEOUT_MS));
	}

	public boolean setNominalOutputReverse(final double percentOut) {
		return runWithRetries(() -> m_internal.configNominalOutputReverse(percentOut, TIMEOUT_MS));
	}

	public boolean setPeakOutputForward(final double percentOut) {
		return runWithRetries(() -> m_internal.configPeakOutputForward(percentOut, TIMEOUT_MS));
	}

	public boolean setPeakOutputReverse(final double percentOut) {
		return runWithRetries(() -> m_internal.configPeakOutputReverse(percentOut, TIMEOUT_MS));
	}

	public boolean setNominalOutputs(final double forward, final double reverse) {
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

	private double meters2NativeUnits(final double meters) {
		return rotations2NativeUnits(meters / (Tau * m_cylinderRadiusMeters) * 60);
	}

	private double nativeUnits2Rotations(final double nativeUnits) {
		return nativeUnits / m_ticksPerRotation;
	}

	private double nativeUnits2RPM(final double nativeUnits) {
		return nativeUnits / m_ticksPerRotation * 600.0;
	}

	private int rotations2NativeUnits(final double rotations) {
		return (int) (rotations * m_ticksPerRotation);
	}

	private int RPM2NativeUnitsPer100ms(final double rpm) {
		return (int) (rpm * m_ticksPerRotation / 600.0);
	}

	private synchronized boolean runWithRetries(final Supplier<ErrorCode> call) {
		boolean success;

		int tries = 0;

		do {
			success = call.get() == ErrorCode.OK;
		} while (!success && tries++ < MAX_TRIES);

		if (tries >= MAX_TRIES || !success) {
			report(ReportingEndpoint.Event.Kind.ERROR, "Failed to configure TalonSRX on Port " + m_internal.getDeviceID() + "!!");
			return false;
		} else {
			return true;
		}
	}
}
