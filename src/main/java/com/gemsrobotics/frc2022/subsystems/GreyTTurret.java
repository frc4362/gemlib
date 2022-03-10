package com.gemsrobotics.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.controls.PIDFController;
import com.gemsrobotics.lib.data.InterpolatingTreeMap;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.math.interpolation.InterpolatingDouble;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.subsystems.Turret;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Objects;

import static com.gemsrobotics.lib.utils.MathUtils.Tau;
import static java.lang.Math.abs;

public final class GreyTTurret extends Subsystem implements Turret {
	private static final double REDUCTION = 1.0 / 140.0;
	private static final double ENCODER_COUNTS_PER_REVOLUTION = 2048.0; // unused, integrated sensor
	private static final MotorController.GearingParameters GEARING_PARAMETERS =
			new MotorController.GearingParameters(REDUCTION, 1.0, ENCODER_COUNTS_PER_REVOLUTION);
	private static final PIDFController.Gains GAINS = new PIDFController.Gains(0.1, 0.0, 0.75, 0.0);
	private static final int MOTOR_PORT = 4;
	private static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(0.59621, 2.3639, 0.056218);
	private static final double TICKS_PER_DEGREE = (286720.0 / 360.0);
	private static final double ALLOWABLE_ERROR_TICKS = TICKS_PER_DEGREE * 0.0;
	private static final int STATE_ESTIMATOR_MAX_SAMPLES = 100;
	private static final double TIME_TO_RAMP = 0.01;
	private static final double SOFT_LIMIT_TICKS = TICKS_PER_DEGREE * 178.0;

	private static GreyTTurret INSTANCE;

	public static GreyTTurret getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new GreyTTurret();
		}

		return INSTANCE;
	}

	public final MotorController<TalonFX> m_motor;
	private final InterpolatingTreeMap<InterpolatingDouble, Rotation> m_turretStateEstimator;
	private final PeriodicIO m_periodicIO;

	private Mode m_mode;

	private GreyTTurret() {
		m_motor = MotorControllerFactory.createTalonFX(MOTOR_PORT, MotorControllerFactory.HIGH_PERFORMANCE_TALON_CONFIG, false);
		m_motor.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		m_motor.getInternalController().configVoltageCompSaturation(10.0);
		m_motor.setGearingParameters(GEARING_PARAMETERS);
		m_motor.setPIDF(GAINS);
		m_motor.setInvertedOutput(true);
		m_motor.setSelectedProfile(0);
		m_motor.setEncoderCounts(0.0);

		// soft limits
		m_motor.getInternalController().configForwardSoftLimitEnable(true);
		m_motor.getInternalController().configForwardSoftLimitThreshold(SOFT_LIMIT_TICKS);
		m_motor.getInternalController().configReverseSoftLimitEnable(true);
		m_motor.getInternalController().configReverseSoftLimitThreshold(-SOFT_LIMIT_TICKS);

		// acceptable error
		m_motor.getInternalController().configAllowableClosedloopError(0, ALLOWABLE_ERROR_TICKS);
		m_motor.getInternalController().configNominalOutputForward(FEEDFORWARD.ks / 12.0);
		m_motor.getInternalController().configNominalOutputReverse(-FEEDFORWARD.ks / 12.0);
		m_motor.getInternalController().configPeakOutputForward(1.0);
		m_motor.getInternalController().configPeakOutputReverse(-1.0);
		m_motor.setClosedLoopVoltageRampRate(TIME_TO_RAMP);

		m_turretStateEstimator = new InterpolatingTreeMap<>(STATE_ESTIMATOR_MAX_SAMPLES);

		m_periodicIO = new PeriodicIO();
	}

	public enum Mode {
		DISABLED,
		ROTATION
	}

	private static class PeriodicIO {
		public double currentAmps = 0.0;
		public Rotation reference = Rotation.identity();
		public Rotation position = Rotation.identity();
		public Rotation velocity = Rotation.identity();
	}

	@Override
	protected void readPeriodicInputs(final double timestamp) {
		m_periodicIO.currentAmps = m_motor.getDrawnCurrentAmps();

		final var oldPosition = new Rotation(m_periodicIO.position);
		m_periodicIO.position = Rotation.radians(m_motor.getPositionRotations() * Tau);

		final var currentVelocity = m_periodicIO.position.difference(oldPosition).getRadians() / dt();
		m_periodicIO.velocity = Rotation.radians(currentVelocity);

		m_turretStateEstimator.put(new InterpolatingDouble(timestamp), m_periodicIO.position);
	}

	@Override
	protected void onStart(final double timestamp) {
		setDisabled();
		m_motor.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
	}

	@Override
	protected void onUpdate(final double timestamp) {
		switch(m_mode) {
			case DISABLED:
				m_motor.setNeutral();
				break;
			case ROTATION:
				m_motor.setPositionRotations(m_periodicIO.reference.getRadians() / Tau);
				break;
		}
	}

	public void setDisabled() {
		m_mode = Mode.DISABLED;
	}

	@Override
	public void setReference(Rotation reference) {
		m_mode = Mode.ROTATION;
		m_periodicIO.reference = reference;
	}

	@Override
	public boolean atReference() {
		return abs(m_motor.getInternalController().getClosedLoopError()) < ALLOWABLE_ERROR_TICKS;
	}

	@Override
	public Rotation getRotation() {
		return m_periodicIO.position;
	}

	// May be exact
	public Rotation getEstimatedRotation(final double timestamp) {
		return m_turretStateEstimator.getInterpolated(new InterpolatingDouble(timestamp));
	}

	public Rotation getReference() {
		return m_periodicIO.reference;
	}

	@Override
	protected void onStop(double timestamp) {
		setDisabled();
		m_motor.setNeutralBehaviour(MotorController.NeutralBehaviour.COAST);
	}

	@Override
	public void setSafeState() {
		m_motor.setNeutralBehaviour(MotorController.NeutralBehaviour.COAST);
		m_motor.setNeutral();
	}
}
