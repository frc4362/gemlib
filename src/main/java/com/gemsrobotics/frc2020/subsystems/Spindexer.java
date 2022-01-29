package com.gemsrobotics.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.frc2020.Constants;
import com.gemsrobotics.lib.data.RollingAverageDouble;
import com.gemsrobotics.lib.drivers.motorcontrol.GemTalon;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.utils.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.Loggable;

import java.util.Objects;

import static java.lang.Math.*;

public final class Spindexer extends Subsystem {
	private static final double STICTION = 0.6 / 12.0; // actually .6
	private static final MotorController.MotionParameters MOTION_PARAMETERS =
			new MotorController.MotionParameters(4800, 180, 0.08);
	private static final MotorController.GearingParameters GEARING_PARAMETERS =
			new MotorController.GearingParameters(1.0 / 30.0, Units.inches2Meters(13.75) / 2.0, 1.0);

	private static Spindexer INSTANCE;

	public static Spindexer getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Spindexer();
		}

		return INSTANCE;
	}

	private final GemTalon<TalonFX> m_motor;
	private final RollingAverageDouble m_stallAverage;
	private final PeriodicIO m_periodicIO;

	private Mode m_mode;

	private Spindexer() {
		m_motor = MotorControllerFactory.createDefaultTalonFX(Constants.HOPPER_PORT);
		m_motor.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		m_motor.setGearingParameters(GEARING_PARAMETERS);
		m_motor.setPeakOutputs(0.33, -0.33);
		m_motor.setSelectedProfile(0);
		m_motor.setPIDF(0.4, 0.0, 0.842, 0.0);
		m_motor.setInvertedOutput(false);
		m_motor.setEncoderCounts(0.0);

		m_stallAverage = new RollingAverageDouble(40);

		m_periodicIO = new PeriodicIO();

		m_mode = Mode.DISABLED;
	}

	public enum Mode {
		DISABLED,
		POSITION,
		AUTO_SORT
	}

	private static class PeriodicIO implements Loggable {
		public double referenceRotations = 0.0;
		public double positionRotations = 0.0;
		public double velocityRPM = 0.0;
		public boolean atReference = false;
	}

	@Override
	protected synchronized void readPeriodicInputs(final double timestamp) {
		m_periodicIO.positionRotations = m_motor.getPositionRotations();
		m_periodicIO.velocityRPM = m_motor.getVelocityAngularRPM();
		m_periodicIO.atReference = abs(m_periodicIO.referenceRotations - m_periodicIO.positionRotations) < (0.6 / 360.0)
								   && abs(m_periodicIO.velocityRPM) < 0.05;
	}

	public synchronized void rotate(final double steps) {
		final var change = (1.0 / 6.0) * -steps;

		if (m_mode == Mode.AUTO_SORT) {
			m_periodicIO.referenceRotations = m_periodicIO.positionRotations + change;
		} else {
			m_periodicIO.referenceRotations = m_periodicIO.referenceRotations + change;
		}

		m_periodicIO.atReference = false;
		m_mode = Mode.POSITION;
	}

	public synchronized void setShootingPosition() {
		m_periodicIO.referenceRotations = round(m_periodicIO.positionRotations * 6.0) / 6.0;
		m_periodicIO.atReference = false;
		m_mode = Mode.POSITION;
	}

	public synchronized void setSorting() {
		m_mode = Mode.AUTO_SORT;
	}

	public synchronized void setDisabled() {
		m_mode = Mode.DISABLED;
	}

	@Override
	protected synchronized void onStart(final double timestamp) {
	}

	@Override
	protected synchronized void onUpdate(final double timestamp) {
		SmartDashboard.putNumber("Spindexer Current", m_motor.getDrawnCurrentAmps());
		SmartDashboard.putNumber("Spindexer Velocity", m_motor.getVelocityAngularRPM());

		switch (m_mode) {
			case POSITION:
				m_stallAverage.clear();

				final var error = m_motor.getInternalController().getClosedLoopError();
				final var deadbandedError = abs(error) > 10.0 ? error : 0.0;
				m_motor.setPositionRotations(m_periodicIO.referenceRotations, signum(deadbandedError) * STICTION);

				if (m_periodicIO.atReference) {
					m_mode = Mode.DISABLED;
				}

				break;
			case AUTO_SORT:
				m_stallAverage.add(m_motor.getDrawnCurrentAmps());

				if (m_stallAverage.getAverage() > 47.0 && !m_stallAverage.hasSpaceRemaining()) {
					m_motor.setVoltage(-2.75);
				} else {
					m_motor.setVoltage(1.815);
				}

				break;
			default:
				m_stallAverage.clear();
				m_motor.setDutyCycle(0.0);
				break;
		}
	}

	@Override
	protected synchronized void onStop(final double timestamp) {
	}

	@Override
	public void setSafeState() {
		m_motor.setDutyCycle(0.0);
	}

	public synchronized boolean atRest() {
		return m_periodicIO.atReference;
	}

	public synchronized double getRotations() {
		return m_periodicIO.positionRotations;
	}

	public synchronized Mode getState() {
		return m_mode;
	}
}
