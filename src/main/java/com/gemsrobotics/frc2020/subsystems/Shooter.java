package com.gemsrobotics.frc2020.subsystems;

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
	private static final PIDFController.Gains SHOOTER_GAINS = new PIDFController.Gains(0.156, 0.0, 0.0, 0.0);
	private static final PIDFController.Gains KICKER_GAINS = new PIDFController.Gains(0.0, 0.0, 0.0, 0.0);

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
			m_kickerMaster,
			m_kickerSlave;
	private final MotorFeedforward m_shooterFeedforward;
	private final PeriodicIO m_periodicIO;

	private Shooter() {
		m_shooterMaster = MotorControllerFactory.createDefaultTalonFX(5);
		m_shooterMaster.setGearingParameters(1.0, SHOOTER_WHEEL_RADIUS);
		m_shooterMaster.setNeutralBehaviour(MotorController.NeutralBehaviour.COAST);
		m_shooterMaster.setInvertedOutput(false);
		m_shooterMaster.setSelectedProfile(0);
		m_shooterMaster.setPIDF(SHOOTER_GAINS);

		m_shooterSlave = MotorControllerFactory.createSlaveTalonFX(6);
		m_shooterSlave.setGearingParameters(0.0, SHOOTER_WHEEL_RADIUS);
		m_shooterSlave.setNeutralBehaviour(MotorController.NeutralBehaviour.COAST);
		m_shooterSlave.follow(m_shooterMaster, true);

		m_kickerMaster = MotorControllerFactory.createDefaultTalonFX(7);
		m_kickerMaster.setNeutralBehaviour(MotorController.NeutralBehaviour.COAST);
		m_kickerMaster.setGearingParameters(1.0, SHOOTER_WHEEL_RADIUS);
		m_kickerMaster.setSelectedProfile(0);
		m_kickerMaster.setPIDF(KICKER_GAINS);
		m_kickerMaster.setInvertedOutput(true);

		m_kickerSlave = MotorControllerFactory.createSlaveTalonFX(8);
		m_kickerSlave.setGearingParameters(1.0, SHOOTER_WHEEL_RADIUS);
		m_kickerSlave.setNeutralBehaviour(MotorController.NeutralBehaviour.COAST);
		m_kickerSlave.follow(m_kickerMaster, true);

		m_shooterFeedforward = new MotorFeedforward(0.375, 0.0018, 7.116666666666667E-5);

		m_periodicIO = new PeriodicIO();
	}

	private static class PeriodicIO implements Loggable {
		@Log(name="Shooter Velocity (RPS)")
		public double shooterMeasuredRPM = 0.0;
		@Log(name="Shooter Reference (RPS)")
		public double shooterReferenceRPM = 0.0;
		@Log(name="Shooter Current Draw (amps)")
		public double shooterCurrent = 0.0;
		@Log(name="Kicker Velocity (RPM)")
		public double kickerRPM = 0.0;
		@Log(name="Kicker Reference (RPM)")
		public double kickerReference = 0.0;
		@Log(name="Kicker Current Draw (amps)")
		public double kickerCurrent = 0.0;
	}

	@Override
	protected synchronized void readPeriodicInputs(final double timestamp) {
		m_periodicIO.shooterMeasuredRPM = m_shooterMaster.getVelocityAngularRPM();
		m_periodicIO.shooterCurrent = m_shooterMaster.getDrawnCurrent() + m_shooterSlave.getDrawnCurrent();
//		m_periodicIO.kickerRPM = m_kickerMaster.getVelocityAngularRPM() / 60.0;
//		m_periodicIO.kickerCurrent = m_kickerMaster.getDrawnCurrent() + m_kickerSlave.getDrawnCurrent();
	}

	public synchronized void setRPMs(final double shooterRPM, final double kickerRPM) {
		m_periodicIO.shooterReferenceRPM = shooterRPM;
		m_periodicIO.kickerReference = kickerRPM;
	}

	public synchronized void setDisabled() {
		setRPMs(0, 0);
	}

	@Override
	protected synchronized void onCreate(final double timestamp) {
		setDisabled();
	}

	@Override
	protected synchronized void onUpdate(final double timestamp) {
//		m_kickerMaster.setVelocityRPM(m_periodicIO.kickerRPM);

		final double velocityTarget = m_periodicIO.shooterReferenceRPM;
		final double accelerationTarget = (velocityTarget - m_periodicIO.shooterMeasuredRPM) / dt();
		final double feedforward = m_shooterFeedforward.calculateVolts(velocityTarget, accelerationTarget) / 12.0;
		m_shooterMaster.setVelocityRPM(velocityTarget, feedforward);
	}

	@Override
	protected synchronized void onStop(final double timestamp) {
		setDisabled();
	}

	@Override
	public void setSafeState() {
		m_shooterMaster.setNeutral();
		m_kickerMaster.setNeutral();
	}

	public synchronized boolean atReference(final double thresholdRPM) {
		return epsilonEquals(m_periodicIO.shooterMeasuredRPM, m_periodicIO.shooterReferenceRPM, thresholdRPM)
			   && epsilonEquals(m_periodicIO.kickerRPM, m_periodicIO.kickerReference, thresholdRPM);
	}
}
