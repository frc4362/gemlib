package com.gemsrobotics.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.frc2020.Constants;
import com.gemsrobotics.lib.controls.MotorFeedforward;
import com.gemsrobotics.lib.controls.PIDFController;
import com.gemsrobotics.lib.data.LimitedQueue;
import com.gemsrobotics.lib.data.RollingAverageDouble;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.utils.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.Objects;

import static com.gemsrobotics.lib.utils.MathUtils.epsilonEquals;

public final class Shooter extends Subsystem implements Loggable {
	private static final double SHOOTER_WHEEL_RADIUS = Units.inches2Meters(3.8) / 2.0;
	private static final PIDFController.Gains SHOOTER_GAINS = new PIDFController.Gains(0.293, 0.0, 0.0, 0.0);
	private static final PIDFController.Gains FEEDER_GAINS = new PIDFController.Gains(0.0455, 0.0, 0.0, 0.0);
	private static final int RPM_SAMPLE_SIZE = 5;

	private static Shooter INSTANCE;

	public static Shooter getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Shooter();
		}

		return INSTANCE;
	}

	private final MotorController<TalonFX>
			m_shooterMaster,
			m_shooterSlave,
			m_feederMaster,
			m_feederSlave;
	private final MotorFeedforward
			m_shooterFeedforward,
			m_feederFeedforward;
	private final LimitedQueue<Double> m_shooterSamples, m_feederSamples;
	private final PeriodicIO m_periodicIO;

	private Shooter() {
		m_shooterMaster = MotorControllerFactory.createDefaultTalonFX(Constants.SHOOTER_MASTER_PORT);
		m_shooterMaster.setGearingParameters(1.0, SHOOTER_WHEEL_RADIUS, 2048);
		m_shooterMaster.setNeutralBehaviour(MotorController.NeutralBehaviour.COAST);
		m_shooterMaster.setInvertedOutput(false);
		m_shooterMaster.setSelectedProfile(0);
		m_shooterMaster.setPIDF(SHOOTER_GAINS);

		m_shooterSlave = MotorControllerFactory.createDefaultTalonFX(Constants.SHOOTER_SLAVE_PORT);
		m_shooterSlave.setGearingParameters(1.0, SHOOTER_WHEEL_RADIUS, 2048);
		m_shooterSlave.setNeutralBehaviour(MotorController.NeutralBehaviour.COAST);
		m_shooterSlave.follow(m_shooterMaster, true);

		m_feederMaster = MotorControllerFactory.createDefaultTalonFX(Constants.FEEDER_MASTER_PORT);
		m_feederMaster.setNeutralBehaviour(MotorController.NeutralBehaviour.COAST);
		m_feederMaster.setGearingParameters(1.5, SHOOTER_WHEEL_RADIUS, 2048);
		m_feederMaster.setSelectedProfile(0);
		m_feederMaster.setPIDF(FEEDER_GAINS);
		m_feederMaster.setInvertedOutput(false);

		m_feederSlave = MotorControllerFactory.createDefaultTalonFX(Constants.FEEDER_SLAVE_PORT);
		m_feederSlave.setGearingParameters(1.5, SHOOTER_WHEEL_RADIUS, 2048);
		m_feederSlave.setNeutralBehaviour(MotorController.NeutralBehaviour.COAST);
		m_feederSlave.follow(m_feederMaster, false);

		m_shooterFeedforward = new MotorFeedforward(0.0249, 0.112 / 60.0, 0.0 / 60.0); // 0.0001
		m_feederFeedforward = new MotorFeedforward(0.0141, 0.11 / 60.0, 0.0);

		m_shooterSamples = new LimitedQueue<>(RPM_SAMPLE_SIZE);
		m_feederSamples = new LimitedQueue<>(RPM_SAMPLE_SIZE);

		m_periodicIO = new PeriodicIO();
	}

	private static class PeriodicIO implements Loggable {
		@Log(name="Shooter Velocity (RPM)")
		public double shooterMeasuredRPM = 0.0;
		@Log(name="Shooter Reference (RPM)")
		public double shooterReferenceRPM = 0.0;
		@Log(name="Shooter Current Draw (amps)")
		public double shooterCurrent = 0.0;
		@Log(name="Feeder Velocity (RPM)")
		public double feederMeasuredRPM = 0.0;
		@Log(name="Feeder Reference (RPM)")
		public double feederReferenceRPM = 0.0;
		@Log(name="Feeder Current Draw (amps)")
		public double feederCurrent = 0.0;
		@Log(name="Enabled?")
		public boolean enabled = false;
	}

	@Override
	protected synchronized void readPeriodicInputs(final double timestamp) {
		m_periodicIO.shooterMeasuredRPM = m_shooterMaster.getVelocityAngularRPM();
		m_periodicIO.shooterCurrent = m_shooterMaster.getDrawnCurrent() + m_shooterSlave.getDrawnCurrent();
		m_periodicIO.feederMeasuredRPM = m_feederMaster.getVelocityAngularRPM();
		m_periodicIO.feederCurrent = m_feederMaster.getDrawnCurrent() + m_feederSlave.getDrawnCurrent();
	}

	public synchronized void setRPM(final double shooterRPM) {
		m_periodicIO.enabled = shooterRPM != 0.0;
		m_periodicIO.shooterReferenceRPM = shooterRPM;
		m_periodicIO.feederReferenceRPM = shooterRPM / 1.75;
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
			m_feederSamples.add(m_periodicIO.feederMeasuredRPM);

			final double kickerFeedforward = m_feederFeedforward.calculateVolts(m_periodicIO.feederReferenceRPM) / 12.0;
			m_feederMaster.setVelocityRPM(m_periodicIO.feederReferenceRPM, kickerFeedforward);

			m_shooterSamples.add(m_periodicIO.shooterMeasuredRPM);

			final double accelerationSetpoint = (m_periodicIO.shooterReferenceRPM - m_periodicIO.shooterMeasuredRPM) / dt();
			final double shooterFeedforward = m_shooterFeedforward.calculateVolts(m_periodicIO.shooterReferenceRPM, accelerationSetpoint) / 12.0;
			m_shooterMaster.setVelocityRPM(m_periodicIO.shooterReferenceRPM, shooterFeedforward);
		} else {
			m_feederMaster.setNeutral();
			m_shooterMaster.setNeutral();
		}

		SmartDashboard.putNumber("Shooter Measured RPM", m_periodicIO.shooterMeasuredRPM);
		SmartDashboard.putNumber("Feeder Measured RPM", m_periodicIO.feederMeasuredRPM);
	}

	@Override
	protected synchronized void onStop(final double timestamp) {
		setDisabled();
	}

	@Override
	public void setSafeState() {
		m_shooterMaster.setNeutral();
		m_feederMaster.setNeutral();
	}

	public synchronized boolean atReference() {
		return m_shooterSamples.stream().allMatch(rpm -> epsilonEquals(rpm, m_periodicIO.shooterReferenceRPM, 1000.0))
			   && m_feederSamples.stream().allMatch(rpm -> epsilonEquals(rpm, m_periodicIO.feederReferenceRPM, 1000.0));
	}

	public synchronized boolean isNeutral() {
		return m_periodicIO.shooterReferenceRPM == 0.0;
	}
}
