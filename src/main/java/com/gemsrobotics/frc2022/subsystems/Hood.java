package com.gemsrobotics.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.frc2022.Blackbird;
import com.gemsrobotics.lib.controls.PIDFController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.utils.MathUtils;
import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Objects;

import static com.gemsrobotics.lib.utils.MathUtils.*;
import static java.lang.Math.abs;

public final class Hood extends Subsystem {
	private static final int MOTOR_PORT = 15;
	private static final double REDUCTION = 1.0 / (84.2 * 4.0); // added another gearing stage :)
	private static final int ENCODER_COUNTS_PER_REVOLUTION = 2048;
	private static final MotorController.GearingParameters GEARING =
		new MotorController.GearingParameters(REDUCTION, 1.0, ENCODER_COUNTS_PER_REVOLUTION);
	private static final PIDController CONTROLLER =
			new PIDController(0.4, 0.0, 0.0, Blackbird.kPeriod);
	public static final Rotation
		MIN_ANGLE = Rotation.degrees(22.0),
		MAX_ANGLE = Rotation.degrees(46.0);
	private static final double
		FORWARD_SOFT_LIMIT = 47_200,
		REVERSE_SOFT_LIMIT = -0;
	private static final double STICTION_VOLTS = 0.3;
	private static final double TICKS_PER_DEGREE = FORWARD_SOFT_LIMIT / (MAX_ANGLE.difference(MIN_ANGLE).getDegrees());
	private static final Rotation ACCEPTABLE_ERROR = Rotation.degrees(0.1);

	private static Hood INSTANCE;

	public static Hood getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Hood();
		}

		return INSTANCE;
	}

	private final MotorController<TalonFX> m_motor;
	private final PeriodicIO m_periodicIO;

	private Hood() {
		m_motor = MotorControllerFactory.createTalonFX(MOTOR_PORT, MotorControllerFactory.HOOD_TALON_CONFIG, false);
		m_motor.setGearingParameters(GEARING);
		m_motor.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		m_motor.setInvertedOutput(true);
		m_motor.setOpenLoopVoltageRampRate(0.02);

		m_motor.getInternalController().configAllowableClosedloopError(0, TICKS_PER_DEGREE * ACCEPTABLE_ERROR.getDegrees());
		m_motor.getInternalController().configClosedloopRamp(0.05);

		m_motor.getInternalController().configNominalOutputForward(STICTION_VOLTS / 12.0);
		m_motor.getInternalController().configNominalOutputReverse(-STICTION_VOLTS / 12.0);
		m_motor.getInternalController().configPeakOutputForward(1.0);
		m_motor.getInternalController().configPeakOutputReverse(-1.0);

		// soft limits
		m_motor.getInternalController().configForwardSoftLimitEnable(true);
		m_motor.getInternalController().configForwardSoftLimitThreshold(FORWARD_SOFT_LIMIT);
		m_motor.getInternalController().configReverseSoftLimitEnable(true);
		m_motor.getInternalController().configReverseSoftLimitThreshold(REVERSE_SOFT_LIMIT);

		m_periodicIO = new PeriodicIO();
	}

	private static class PeriodicIO {
		public boolean enabled = false;
		public Rotation reference = Rotation.identity();
		public Rotation position = Rotation.identity();
	}

	@Override
	protected void readPeriodicInputs(final double timestamp) {
		m_periodicIO.position = MIN_ANGLE.rotateBy(Rotation.radians(m_motor.getPositionRotations() * MathUtils.Tau));
	}

	@Override
	protected void onStart(final double timestamp) {

	}

	@Override
	protected void onUpdate(final double timestamp) {
		final var effort = CONTROLLER.calculate(m_periodicIO.position.getDegrees(), m_periodicIO.reference.getDegrees());
		final var errorDegs = m_periodicIO.reference.difference(m_periodicIO.position).getDegrees();
		SmartDashboard.putNumber("Hood Error Degrees", errorDegs);
		SmartDashboard.putNumber("Control Effort", effort);
		if (m_periodicIO.enabled) {
//			m_motor.setPositionRotations(m_periodicIO.reference.difference(MIN_ANGLE).getRadians() / Tau);
			// if (abs(errorDegs) < ACCEPTABLE_ERROR.getDegrees()) {
			// 	m_motor.setNeutral();
			// } else {
				m_motor.setVoltage(effort);
			// }
		} else {
			m_motor.setNeutral();
		}
	}

	@Override
	protected void onStop(final double timestamp) {
		m_motor.setNeutral();
	}

	@Override
	public void setSafeState() {

	}

	public void setDisabled() {
		m_periodicIO.enabled = false;
	}

	public void setReference(Rotation newReference) {
		newReference = Rotation.radians(coerce(
				MIN_ANGLE.getRadians(),
				newReference.getRadians(),
				MAX_ANGLE.getRadians()));

		m_periodicIO.enabled = true;
		m_periodicIO.reference = newReference;
	}

	public void setStowed() {
		setReference(MIN_ANGLE);
	}

	public boolean atReference() {
		return epsilonEquals(
				m_periodicIO.position.getDegrees(),
				m_periodicIO.reference.getDegrees(),
				0.25);
	}
}