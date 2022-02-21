package com.gemsrobotics.lib.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.controls.MotorFeedforward;
import com.gemsrobotics.lib.controls.PIDFController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerGroup;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.utils.Units;
import edu.wpi.first.wpilibj.MedianFilter;

import static com.gemsrobotics.lib.utils.MathUtils.epsilonEquals;

public abstract class Flywheel<MotorType> extends Subsystem {
	private static final int FILTER_SAMPLE_SIZE = 10;

	public static class Config {
		public double wheelRadius;
		public MotorController.GearingParameters gearing;
		public PIDFController.Gains gains;
		public MotorFeedforward feedforward;
		public double allowableRPMError;
	}

	protected abstract Config getConfig();

	/**
	 * Tell the slaves to follow the master BEFORE returning the group.
	 * @return The configured motor group.
	 */
	protected abstract MotorControllerGroup<TalonFX> getMotors();

	private final Config m_config;
	private final MotorControllerGroup<TalonFX> m_motors;
	private final MedianFilter m_filter;
	private final PeriodicIO m_periodicIO;

	protected Flywheel() {
		m_config = getConfig();
		m_motors = getMotors();
		final var motor = m_motors.getMaster();
		motor.setSelectedProfile(0);
		motor.setPIDF(m_config.gains);
		motor.setGearingParameters(1.0, m_config.wheelRadius, 2048);
		motor.setNeutralBehaviour(MotorController.NeutralBehaviour.COAST);

		m_filter = new MedianFilter(FILTER_SAMPLE_SIZE);
		m_periodicIO = new PeriodicIO();
	}

	private static class PeriodicIO {
		public double shooterMeasuredRPM = 0.0;
		public double shooterFilteredRPM = 0.0;
		public double shooterReferenceRPM = 0.0;
		public double shooterCurrent = 0.0;
		public boolean enabled = false;
		public boolean atReference = false;
	}

	@Override
	protected synchronized void readPeriodicInputs(final double timestamp) {
		m_periodicIO.shooterMeasuredRPM = m_motors.getMaster().getVelocityAngularRPM();
		final var masterCurrent = m_motors.getMaster().getDrawnCurrentAmps();
		final var slaveCurrents = m_motors.getSlaves().stream().mapToDouble(MotorController::getDrawnCurrentAmps).sum();
		m_periodicIO.shooterCurrent = masterCurrent + slaveCurrents;
		m_periodicIO.shooterFilteredRPM = m_filter.calculate(m_periodicIO.shooterMeasuredRPM);
	}

	public synchronized void setDisabled() {
		setRPM(0);
	}

	@Override
	protected synchronized void onStart(final double timestamp) {
		setDisabled();
	}

	@Override
	protected synchronized void onUpdate(final double timestamp) {
		if (m_periodicIO.enabled) {
			final double accelerationSetpoint = (m_periodicIO.shooterReferenceRPM - m_periodicIO.shooterFilteredRPM) / dt();
			final double shooterFeedforward = m_config.feedforward.calculateVolts(
					Units.rpm2RadsPerSecond(m_periodicIO.shooterReferenceRPM),
					Units.rpm2RadsPerSecond(accelerationSetpoint)) / 12.0;
			m_motors.getMaster().setVelocityRPM(m_periodicIO.shooterReferenceRPM, shooterFeedforward);
		} else {
			m_motors.getMaster().setNeutral();
		}
	}

	@Override
	protected synchronized void onStop(final double timestamp) {
		setDisabled();
	}

	@Override
	public void setSafeState() {
		m_motors.getMaster().setNeutral();
	}

	public synchronized void setRPM(final double shooterRPM) {
		m_periodicIO.enabled = shooterRPM != 0.0;
		m_periodicIO.shooterReferenceRPM = shooterRPM;
	}

	public synchronized void setMetersPerSecond(final double shooterVelocity) {
		m_periodicIO.enabled = shooterVelocity != 0.0;
		m_periodicIO.shooterReferenceRPM = Units.radsPerSec2Rpm(shooterVelocity / m_config.wheelRadius);
	}

	public synchronized boolean atReference() {
		return epsilonEquals(m_periodicIO.shooterReferenceRPM, m_periodicIO.shooterFilteredRPM, m_config.allowableRPMError);
	}

	public synchronized boolean isNeutral() {
		return m_periodicIO.shooterReferenceRPM == 0.0;
	}
}
