package com.gemsrobotics.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.controls.MotorFeedforward;
import com.gemsrobotics.lib.controls.PIDFController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.utils.Units;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.Objects;

import static com.gemsrobotics.lib.utils.MathUtils.epsilonEquals;

public final class Shooter extends Subsystem implements Loggable {
	private static final double SHOOTER_WHEEL_RADIUS = Units.inches2Meters(3.8) / 2.0;
	private static final PIDFController.Gains SHOOTER_GAINS = new PIDFController.Gains(0.28, 0.0, 0.0, 0.0);
	private static final PIDFController.Gains FEEDER_GAINS = new PIDFController.Gains(0.113, 0.0, 0.0, 0.0);

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
	private final PeriodicIO m_periodicIO;

	private Shooter() {
		m_shooterMaster = MotorControllerFactory.createDefaultTalonFX(5);
		m_shooterMaster.getInternalController().configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		m_shooterMaster.setGearingParameters(1.0, SHOOTER_WHEEL_RADIUS, 2048);
		m_shooterMaster.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		m_shooterMaster.setInvertedOutput(false);
		m_shooterMaster.setSelectedProfile(0);
		m_shooterMaster.setPIDF(SHOOTER_GAINS);

		m_shooterSlave = MotorControllerFactory.createSlaveTalonFX(6);
		m_shooterMaster.getInternalController().configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		m_shooterSlave.setGearingParameters(1.0, SHOOTER_WHEEL_RADIUS, 2048);
		m_shooterSlave.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		m_shooterSlave.follow(m_shooterMaster, true);

		m_feederMaster = MotorControllerFactory.createDefaultTalonFX(7);
		m_feederMaster.getInternalController().configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		m_feederMaster.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		m_feederMaster.setGearingParameters(1.0, SHOOTER_WHEEL_RADIUS, 2048);
		m_feederMaster.setSelectedProfile(0);
		m_feederMaster.setPIDF(FEEDER_GAINS);
		m_feederMaster.setInvertedOutput(true);

		m_feederSlave = MotorControllerFactory.createSlaveTalonFX(8);
		m_feederSlave.getInternalController().configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		m_feederSlave.setGearingParameters(1.0, SHOOTER_WHEEL_RADIUS, 2048);
		m_feederSlave.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		m_feederSlave.follow(m_feederMaster, true);

		m_shooterFeedforward = new MotorFeedforward(0.323, 0.118 / 60.0, 0.0004 / 60.0);
		m_feederFeedforward = new MotorFeedforward(0.0608, 0.109 / 60.0, 0.0);

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
	}

	@Override
	protected synchronized void readPeriodicInputs(final double timestamp) {
		m_periodicIO.shooterMeasuredRPM = m_shooterMaster.getVelocityAngularRPM();
		m_periodicIO.shooterCurrent = m_shooterMaster.getDrawnCurrent() + m_shooterSlave.getDrawnCurrent();
		m_periodicIO.feederMeasuredRPM = m_feederMaster.getVelocityAngularRPM();
		m_periodicIO.feederCurrent = m_feederMaster.getDrawnCurrent() + m_feederSlave.getDrawnCurrent();
	}

	public synchronized void setRPM(final double shooterRPM) {
		m_periodicIO.shooterReferenceRPM = shooterRPM;
		m_periodicIO.feederReferenceRPM = shooterRPM / 1.75;
	}

	public synchronized void setDisabled() {
		setRPM(0);
	}

	@Override
	protected synchronized void onCreate(final double timestamp) {
		setDisabled();
	}

	@Override
	protected synchronized void onUpdate(final double timestamp) {
		final double kickerFeedforward = m_feederFeedforward.calculateVolts(m_periodicIO.feederReferenceRPM) / 12.0;
		m_feederMaster.setVelocityRPM(m_periodicIO.feederReferenceRPM, kickerFeedforward);

		final double accelerationSetpoint = (m_periodicIO.shooterReferenceRPM - m_periodicIO.shooterMeasuredRPM) / dt();
		final double shooterFeedforward = m_shooterFeedforward.calculateVolts(m_periodicIO.shooterReferenceRPM, accelerationSetpoint) / 12.0;
		m_shooterMaster.setVelocityRPM(m_periodicIO.shooterReferenceRPM, shooterFeedforward);
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

	public synchronized boolean atReference(final double thresholdRPM) {
		return epsilonEquals(m_periodicIO.shooterMeasuredRPM, m_periodicIO.shooterReferenceRPM, thresholdRPM)
			   && epsilonEquals(m_periodicIO.feederMeasuredRPM, m_periodicIO.feederReferenceRPM, thresholdRPM);
	}
}
