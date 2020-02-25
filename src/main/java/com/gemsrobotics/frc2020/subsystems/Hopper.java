package com.gemsrobotics.frc2020.subsystems;

import com.gemsrobotics.frc2020.Constants;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.utils.Units;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import io.github.oblarg.oblog.Loggable;

import java.util.Objects;

import static com.gemsrobotics.lib.utils.MathUtils.epsilonEquals;

public final class Hopper extends Subsystem {
	private static final MotorController.GearingParameters GEARING_PARAMETERS =
			new MotorController.GearingParameters(1.0 / 96.9625, Units.inches2Meters(13.75) / 2.0, 1.0);

	private static Hopper INSTANCE;

	public static Hopper getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Hopper();
		}

		return INSTANCE;
	}

	private final MotorController<CANSparkMax> m_motor;
	private final PeriodicIO m_periodicIO;

	private Mode m_mode;

	private Hopper() {
		m_motor = MotorControllerFactory.createSparkMax(Constants.HOPPER_PORT, MotorControllerFactory.DEFAULT_SPARK_CONFIG);
		m_motor.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		m_motor.setGearingParameters(GEARING_PARAMETERS);
		m_motor.setSelectedProfile(0);
		m_motor.setPIDF(12.1, 0.0, 0.979, 0.0);
		m_motor.setInvertedOutput(true);
		m_motor.setEncoderRotations(0.0);

		m_periodicIO = new PeriodicIO();

		m_mode = Mode.DISABLED;
	}

	public enum Mode {
		DISABLED,
		RATCHETING
	}

	private static class PeriodicIO implements Loggable {
		public double referenceRotations = 0.0;
		public double positionRotations = 0.0;
		public Rotation velocityRotationsPerSecond = Rotation.identity();
		public boolean atReference = false;
	}

	@Override
	protected synchronized void readPeriodicInputs(final double timestamp) {
//		m_periodicIO.observedColor = m_sensor.getRawColor();
		m_periodicIO.positionRotations = m_motor.getPositionRotations();
		m_periodicIO.velocityRotationsPerSecond = Rotation.radians(m_motor.getVelocityAngularRadiansPerSecond());
		m_periodicIO.atReference = epsilonEquals(m_periodicIO.referenceRotations, m_periodicIO.positionRotations, (0.4 / 360.0))
								   && epsilonEquals(m_periodicIO.velocityRotationsPerSecond.getDegrees(), 0.0, 2.0);
	}

	public synchronized void rotate(final int steps) {
		m_mode = Mode.RATCHETING;
		m_periodicIO.referenceRotations = m_periodicIO.referenceRotations + (1.0 / 6.0) * steps;
	}

	public synchronized void setDisabled() {
		m_mode = Mode.DISABLED;
	}

	@Override
	protected synchronized void onStart(final double timestamp) {
	}

	@Override
	protected synchronized void onUpdate(final double timestamp) {
		switch (m_mode) {
			case RATCHETING:
				m_motor.setPositionRotations(m_periodicIO.referenceRotations);

				if (m_periodicIO.atReference) {
					m_mode = Mode.DISABLED;
				}

				break;
			default:
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
}
