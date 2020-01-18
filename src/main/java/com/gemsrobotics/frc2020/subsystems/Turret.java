package com.gemsrobotics.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.gemsrobotics.frc2020.Constants;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.telemetry.reporting.ReportingEndpoint.Event.Kind;
import com.gemsrobotics.lib.utils.MathUtils;
import com.gemsrobotics.lib.utils.TalonUtils;
import edu.wpi.first.wpilibj.Talon;
import io.github.oblarg.oblog.Loggable;

import java.util.Objects;

public final class Turret extends Subsystem {
	private static final double POSITION_PER_ = 1.0;

	private static Turret INSTANCE;

	public Turret getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Turret();
		}

		return INSTANCE;
	}

	private final TalonSRX m_motor;
	private final PeriodicIO m_periodicIO;

	private Mode m_controlMode;

	private Turret() {
		m_motor = new TalonSRX(Constants.TURRET_PORT);
		m_motor.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0, 10);

		m_periodicIO = new PeriodicIO();
	}

	public enum Mode {
		DISABLED,
		ROTATION
	}

	private static class PeriodicIO implements Loggable {
		public double reference = 0.0;
		public double position = 0.0;
		public double velocity = 0.0;
		public double acceleration = 0.0;
	}

	public synchronized void setDisabled() {
		m_controlMode = Mode.DISABLED;
	}

	public synchronized void setRotationGoal(final Rotation rotation) {

	}

	@Override
	protected synchronized void readPeriodicInputs(final double timestamp) {
		m_periodicIO.position = m_motor.getSelectedSensorPosition();
		final double oldVelocity = m_periodicIO.velocity;
		final double currentVelocity = m_motor.getSelectedSensorVelocity();
		m_periodicIO.velocity = currentVelocity;
		m_periodicIO.acceleration = (currentVelocity - oldVelocity) / dt();
	}

	@Override
	protected synchronized void onCreate(final double timestamp) {

	}

	@Override
	protected synchronized void onUpdate(final double timestamp) {
		switch (m_controlMode) {
			case DISABLED:
				m_motor.set(ControlMode.PercentOutput, 0.0);
				break;
			case ROTATION:
				m_motor.set(ControlMode.Position, m_periodicIO.reference);
				break;
		}
	}

	@Override
	protected synchronized void onStop(final double timestamp) {
		setDisabled();
	}

	@Override
	public synchronized void setSafeState() {

	}

//	public synchronized Rotation getCurrentRotation() {
//		return m_periodicIO.position
//	}

//	public synchronized boolean atReference(final Rotation tolerance) {
//		return MathUtils.epsilonEquals(m_periodicIO.position % ROTATION_TICKS, 0.0, tolerance);
//	}

	@Override
	public FaultedResponse checkFaulted() {
		if (TalonUtils.isEncoderPresent(m_motor)) {
			return FaultedResponse.NONE;
		} else {
			report(Kind.HARDWARE_FAULT, "Encoder lost!");
			return FaultedResponse.DISABLE_SUBSYSTEM;
		}
	}
}
