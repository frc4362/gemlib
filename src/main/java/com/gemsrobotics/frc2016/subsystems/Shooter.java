package com.gemsrobotics.frc2016.subsystems;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.gemsrobotics.lib.controls.MotorFeedforward;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.utils.MathUtils;
import edu.wpi.first.wpilibj.Timer;
import io.github.oblarg.oblog.Loggable;

public final class Shooter extends Subsystem {
	public static final int FIRING_SPEED = 7000;
	private final MotorController m_motor1, m_motor2;

	private final PeriodicIO m_periodicIO;

	private Shooter() {
		m_motor1 = MotorControllerFactory.createDefaultTalonSRX(7);
		m_motor2 = MotorControllerFactory.createSlaveTalonSRX(8);
		m_motor2.follow(m_motor1, true);

		m_motor1.setSelectedProfile(0);
		m_motor1.setPIDF(0.225, 0.0, 400.0, 0.0);

		m_periodicIO = new PeriodicIO();
	}

	private enum ControlMode {
		DISABLED,
		OPEN_LOOP,
		RPM
	}

	private static class PeriodicIO implements Loggable {
		public ControlMode controlMode;
		public double demand;

		public double timeSpunUp;
		public double rpm;
		public double currentAmps;
	}

	@Override
	protected synchronized void readPeriodicInputs() {
		m_periodicIO.rpm = m_motor1.getVelocityMotorRPM();
		m_periodicIO.currentAmps = m_motor1.getDrawnCurrent();
	}

	@Override
	protected synchronized void onCreate(double timestamp) {

	}

	@Override
	protected synchronized void onEnable(double timestamp) {

	}

	@Override
	protected synchronized void onUpdate(double timestamp) {
		if (m_periodicIO.controlMode == ControlMode.RPM
			&& MathUtils.epsilonEquals(m_periodicIO.rpm, FIRING_SPEED, 100)
		) {
			m_periodicIO.timeSpunUp = timestamp;
		} else {
			m_periodicIO.timeSpunUp = Double.NaN;
		}

		switch (m_periodicIO.controlMode) {
			case DISABLED:
				m_motor1.setDutyCycle(0);
				break;
			case OPEN_LOOP:
				m_motor1.setDutyCycle(m_periodicIO.demand);
				break;
			case RPM:
				final double feedforward = (m_motor1.getVoltageInput() / 11000.0) * m_periodicIO.demand;
				m_motor1.setVelocityMotorRPM(m_periodicIO.demand, feedforward / 12.0);
				break;
		}
	}

	@Override
	protected synchronized void onStop(double timestamp) {

	}

	@Override
	public void setSafeState() {

	}

	public synchronized void setDisabled() {
		m_periodicIO.controlMode = ControlMode.DISABLED;
		m_periodicIO.demand = 0.0;
	}

	public synchronized void setOpenLoop(final double demand) {
		m_periodicIO.controlMode = ControlMode.OPEN_LOOP;
		m_periodicIO.demand = demand;
	}

	private synchronized void setRPM(final double rpm) {
		m_periodicIO.controlMode = ControlMode.RPM;
		m_periodicIO.demand = rpm;
	}

	public synchronized void setFiringSpeed() {
		setRPM(FIRING_SPEED);
	}

	public synchronized boolean isSpunUp() {
		return !Double.isNaN(m_periodicIO.timeSpunUp) && (Timer.getFPGATimestamp() - m_periodicIO.timeSpunUp) > 0.1;
	}
}
