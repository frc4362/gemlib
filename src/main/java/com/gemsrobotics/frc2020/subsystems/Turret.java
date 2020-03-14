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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.Objects;

import static com.gemsrobotics.lib.utils.MathUtils.Tau;
import static com.gemsrobotics.lib.utils.MathUtils.epsilonEquals;
import static java.lang.Math.*;

public final class Turret extends Subsystem implements Loggable {
	private static final double HOME_POSITION = -804.0;
	private static final double COUNTS_PER_DEGREE = 11.03888888888889;
	private static final double STICTION_VOLTS = 0.7 / 12.0; // really .9175
	private static final MotorController.GearingParameters GEARING_PARAMETERS =
			new MotorController.GearingParameters(1.0, Units.inches2Meters(13.75) / 2.0, 4096);
	private static final PIDFController.Gains TURRET_GAINS = new PIDFController.Gains(2.66, 0.0, 0.0, 0.0);
	private static final int TURRET_USABLE_RANGE = (int) (4096 * (178.5 / 360.0));

	private static Turret INSTANCE;

	public static Turret getInstance() {
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
		m_motor.getInternalController().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		m_motor.getInternalController().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 1, 10);
		m_motor.getInternalController().setSensorPhase(true);
		m_motor.setInvertedOutput(false);
		m_motor.setSelectedProfile(0);
		final var home = HOME_POSITION - m_motor.getInternalController().getSelectedSensorPosition(1);
		m_motor.setEncoderRotations(0);
		m_motor.setPIDF(TURRET_GAINS);
		m_motor.getInternalController().configForwardSoftLimitEnable(true);
		m_motor.getInternalController().configForwardSoftLimitThreshold(+TURRET_USABLE_RANGE);
		m_motor.getInternalController().configReverseSoftLimitEnable(true);
		m_motor.getInternalController().configReverseSoftLimitThreshold(-TURRET_USABLE_RANGE);
//		m_motor.setMotionParameters(MOTION_PARAMETERS);

		m_periodicIO = new PeriodicIO();
	}

	public enum Mode {
		DISABLED,
		ROTATION
	}

	private static class PeriodicIO implements Loggable {
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

	public synchronized void setReferenceRotation(Rotation reference) {
		m_controlMode = Mode.ROTATION;

		final double setpoint = reference.getRadians() * (4096.0 / Tau);

		if (abs(setpoint) > TURRET_USABLE_RANGE) {
			reference = Rotation.radians(reference.getRadians() - copySign(Tau, setpoint));
		}

		if (epsilonEquals(reference.getDegrees(), 180.0, 1.0)) {
			reference = Rotation.degrees(copySign(reference.getDegrees(), 179.0));
		}

		m_periodicIO.reference = reference;
	}

	public synchronized void setPositionOffset(final Rotation amount) {
		setReferenceRotation(m_periodicIO.position.sum(amount));
	}

	@Override
	protected synchronized void onStart(final double timestamp) {
		setDisabled();
	}

	@Override
	protected synchronized void onUpdate(final double timestamp) {
		switch (m_controlMode) {
			case DISABLED:
				m_motor.setNeutral();
				break;
			case ROTATION:
				final double error = atReference() ? 0.0 : m_motor.getInternalController().getClosedLoopError();
				m_motor.setPositionRotations(m_periodicIO.reference.getRadians() / Tau, signum(error) * STICTION_VOLTS);
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

	public synchronized Rotation getRotation() {
		return m_periodicIO.position;
	}

	public synchronized boolean atReference() {
		return Math.abs(m_motor.getInternalController().getClosedLoopError()) < 10.0;
	}

	public double getAbsolutePosition() {
		return m_motor.getInternalController().getSelectedSensorPosition(1);
	}

	@Override
	public FaultedResponse checkFaulted() {
		return m_motor.isEncoderPresent() ? FaultedResponse.NONE : FaultedResponse.DISABLE_SUBSYSTEM;
	}
}
