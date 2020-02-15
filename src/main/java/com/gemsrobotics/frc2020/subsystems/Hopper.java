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
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.Loggable;

import java.util.Objects;

public final class Hopper extends Subsystem {
	private static final MotorController.GearingParameters GEARING_PARAMETERS =
			new MotorController.GearingParameters(1.0 / 77.575757, Units.inches2Meters(13.75) / 2.0, 1.0);

	private static Hopper INSTANCE;

	public static Hopper getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Hopper();
		}

		return INSTANCE;
	}

	private final MotorController<CANSparkMax> m_motor;
	private final ColorSensorV3 m_sensor;
	private final PeriodicIO m_periodicIO;

	private Hopper() {
		m_motor = MotorControllerFactory.createSparkMax(Constants.HOPPER_MOTOR_PORT, MotorControllerFactory.DEFAULT_SPARK_CONFIG);
		m_motor.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		m_motor.setGearingParameters(GEARING_PARAMETERS);
		m_motor.setSelectedProfile(0);
		m_motor.setPIDF(12.1, 0.0, 0.979, 0.0);
//		m_motor.setClosedLoopVoltageRampRate(0.1);
		m_motor.setEncoderRotations(0.0);

		m_sensor = new ColorSensorV3(I2C.Port.kMXP);

		m_periodicIO = new PeriodicIO();
	}

	public enum Mode {
		DISABLED,
		RATCHETING,
		SURVEYING
	}

	private static class PeriodicIO implements Loggable {
		public ColorSensorV3.RawColor observedColor = new ColorSensorV3.RawColor(0, 0, 0, 0);
		public Rotation velocityRadiansPerSecond = Rotation.identity();
		public double positionRotations = 0.0;
		public double referenceRotations = 0.0;
	}

	@Override
	protected synchronized void readPeriodicInputs(double timestamp) {
//		m_periodicIO.observedColor = m_sensor.getRawColor();
		m_periodicIO.velocityRadiansPerSecond = Rotation.radians(m_motor.getVelocityAngularRadiansPerSecond());
		m_periodicIO.positionRotations = m_motor.getPositionRotations();
	}

	public synchronized void rotate(final int steps) {
		m_periodicIO.referenceRotations = m_periodicIO.referenceRotations + (1.0 / 6.0) * steps;
	}

	@Override
	protected synchronized void onCreate(double timestamp) {
	}

	@Override
	protected synchronized void onUpdate(double timestamp) {
		m_motor.setPositionRotations(m_periodicIO.referenceRotations);
		SmartDashboard.putNumber("Hopper RPM", m_motor.getVelocityAngularRPM());
	}

	@Override
	protected synchronized void onStop(double timestamp) {

	}

	@Override
	public void setSafeState() {

	}
}
