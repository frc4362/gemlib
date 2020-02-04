package com.gemsrobotics.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.gemsrobotics.frc2020.Constants;
import com.gemsrobotics.lib.controls.PIDFController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController.MotionParameters;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.utils.Units;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.Objects;

import static com.gemsrobotics.lib.utils.MathUtils.Tau;
import static com.gemsrobotics.lib.utils.MathUtils.epsilonEquals;

public final class Turret extends Subsystem implements Loggable {
	private static final MotorController.GearingParameters GEARING_PARAMETERS =
			new MotorController.GearingParameters(1.0, Units.inches2Meters(13.75) / 2.0, 4096);
	private static final PIDFController.Gains TURRET_GAINS = new PIDFController.Gains(4.0, 0.0, 17.5, 0.0);
	private static final double MAX_VELOCITY = 0.82290092569967646639980865120777;
	private static final MotionParameters MOTION_PARAMETERS =
			new MotorController.MotionParameters(MAX_VELOCITY * 1.8, MAX_VELOCITY * 0.98, 0.01);

	private static Turret INSTANCE;

	public Turret getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Turret();
		}

		return INSTANCE;
	}

	private final MotorController<TalonSRX> m_motor;
	private final PeriodicIO m_periodicIO;

	private Mode m_controlMode;

	private Turret() {
		m_motor = MotorControllerFactory.createDefaultTalonSRX(Constants.TURRET_PORT);
		m_motor.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		m_motor.setGearingParameters(GEARING_PARAMETERS);
		m_motor.getInternalController().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		m_motor.setSelectedProfile(0);
		m_motor.setEncoderRotations(0);
		m_motor.setPIDF(TURRET_GAINS);
		m_motor.setMotionParameters(MOTION_PARAMETERS);

		m_periodicIO = new PeriodicIO();
	}

	public enum Mode {
		DISABLED,
		ROTATION
	}

	private static class PeriodicIO implements Loggable {
		@Log
		public double outputDutyCycle = 0.0;
		@Log
		public double current = 0.0;
		@Log.ToString
		public Rotation reference = Rotation.identity();
		@Log.ToString
		public Rotation position = Rotation.identity();
		@Log.ToString
		public Rotation velocity = Rotation.identity();
		@Log.ToString
		public Rotation acceleration = Rotation.identity();
	}

	@Override
	protected synchronized void readPeriodicInputs(final double timestamp) {
		m_periodicIO.current = m_motor.getDrawnCurrent();

		final var oldPosition = new Rotation(m_periodicIO.position);
		final var oldVelocity = new Rotation(m_periodicIO.velocity);
		m_periodicIO.position = Rotation.radians(m_motor.getPositionRotations() * Tau);

		final var currentVelocity = m_periodicIO.position.difference(oldPosition).getRadians() / dt();
		m_periodicIO.velocity = Rotation.radians(currentVelocity);

		final var currentAcceleration = m_periodicIO.velocity.difference(oldVelocity).getRadians() / dt();
		m_periodicIO.acceleration = Rotation.radians(currentAcceleration);
	}

	public synchronized void setDisabled() {
		m_controlMode = Mode.DISABLED;
	}

	public synchronized void setOpenLoop(final double dutyCycle) {
		m_periodicIO.outputDutyCycle = dutyCycle;
	}

	public synchronized void setReferencePosition(final Rotation reference) {
		m_periodicIO.reference = reference;
	}

	@Override
	protected synchronized void onCreate(final double timestamp) {
		setDisabled();
	}

	@Override
	protected synchronized void onUpdate(final double timestamp) {
		switch (m_controlMode) {
			case DISABLED:
				m_motor.setNeutral();
				break;
			case ROTATION:
				m_motor.setPositionRotations(m_periodicIO.reference.getRadians() / Tau);
				break;
		}
	}

	@Override
	protected synchronized void onStop(final double timestamp) {
		setDisabled();
	}

	@Override
	public synchronized void setSafeState() {
		m_motor.setNeutralBehaviour(MotorController.NeutralBehaviour.COAST);
		m_motor.setNeutral();
	}

	public synchronized Rotation getPositionRotations() {
		return m_periodicIO.position;
	}

	public synchronized boolean atReference(final Rotation tolerance) {
		return epsilonEquals(m_periodicIO.position.distance(m_periodicIO.reference), 0.0, tolerance.getRadians());
	}

	@Override
	public FaultedResponse checkFaulted() {
		return FaultedResponse.NONE;
	}
}
