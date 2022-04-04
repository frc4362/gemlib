package com.gemsrobotics.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.utils.MathUtils;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Objects;

import static com.gemsrobotics.lib.utils.MathUtils.coerce;

public final class Hood extends Subsystem {
	private static final int MOTOR_PORT = 15;
	private static final double REDUCTION = 1.0 / 84.2;
	private static final int ENCODER_COUNTS_PER_REVOLUTION = 2048;
	private static final MotorController.GearingParameters GEARING =
		new MotorController.GearingParameters(REDUCTION, 1.0, ENCODER_COUNTS_PER_REVOLUTION);
	private static final PIDController CONTROLLER =
			new PIDController(0.3, 0.0, 0.011);
	private static final Rotation
		MIN_ANGLE = Rotation.degrees(21.2),
		MAX_ANGLE = Rotation.degrees(46.0);
	private static final double
		FORWARD_SOFT_LIMIT = 12_225,
		REVERSE_SOFT_LIMIT = -0; // maybe -12_275
	private static final double STICTION_VOLTS = 0.4;

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
		m_motor = MotorControllerFactory.createDefaultTalonFX(MOTOR_PORT);
		m_motor.setGearingParameters(GEARING);
		m_motor.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		m_motor.setInvertedOutput(true);
		m_motor.setOpenLoopVoltageRampRate(0.05);

		m_motor.getInternalController().configAllowableClosedloopError(0, 70.4);

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
		public double voltsApplied = 0.0;
		public Rotation reference = Rotation.identity();
		public Rotation position = Rotation.identity();
	}

	@Override
	protected void readPeriodicInputs(final double timestamp) {
		m_periodicIO.voltsApplied = m_motor.getVoltageOutput();
		m_periodicIO.position = MIN_ANGLE.rotateBy(Rotation.radians(m_motor.getPositionRotations() * MathUtils.Tau));
	}

	@Override
	protected void onStart(final double timestamp) {

	}

	@Override
	protected void onUpdate(final double timestamp) {
		if (m_periodicIO.enabled) {
			final var v = CONTROLLER.calculate(m_periodicIO.position.getDegrees(), m_periodicIO.reference.getDegrees());
			SmartDashboard.putNumber("Hood Volts Effort", v);
			SmartDashboard.putNumber("Hood Position Degrees", m_periodicIO.position.getDegrees());
			m_motor.setVoltage(v);
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
}
