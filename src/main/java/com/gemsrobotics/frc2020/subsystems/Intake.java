package com.gemsrobotics.frc2020.subsystems;

import com.gemsrobotics.lib.controls.PIDFController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.utils.MathUtils;
import com.gemsrobotics.lib.utils.Units;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;

public final class Intake extends Subsystem {
	private final double
			BOTTOM_TO_TOP_DISTANCE = 1.0,
			TOP_TO_HOPPER_DISTANCE = 0.25;
	private final PIDFController.Gains
			VELOCITY_GAINS = new PIDFController.Gains(0.0, 0.0, 0.0, 0.0),
			POSITION_GAINS = new PIDFController.Gains(0.0, 0.0, 0.0, 0.0);

	private final MotorController<CANSparkMax> m_motor;
	private final CANDigitalInput m_switch;
	private final PeriodicIO m_periodicIO;
	private final Inventory.Location m_location;

	private Mode m_mode;

	public Intake(final Inventory.Location location, final int motorPort) {
		m_location = location;

		m_motor = MotorControllerFactory.createSparkMax(motorPort, MotorControllerFactory.DEFAULT_SPARK_CONFIG);
		m_motor.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		m_motor.setCurrentLimit(20);
		m_motor.setSelectedProfile(0);
		m_motor.setPIDF(VELOCITY_GAINS);
		m_motor.setSelectedProfile(1);
		m_motor.setPIDF(POSITION_GAINS);

		m_switch = m_motor.getInternalController().getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen);

		m_periodicIO = new PeriodicIO();
	}

	public enum Mode {
		VELOCITY,
		POSITION
	}

	private static class PeriodicIO {
		public boolean ballObserved = false;
		public boolean ballImplied = false;
		public boolean isBusy = false;
		public int selectedProfile = 0;
		public double referenceMeters = 0.0;
		public double positionMeters = 0.0;
		public double velocityMetersPerSecond = 0.0;
	}

	private void configControlMode(final Mode newMode) {
		if (newMode != m_mode) {
			switch (newMode) {
				case VELOCITY:
					m_motor.setSelectedProfile(0);
					break;
				case POSITION:
					m_motor.setSelectedProfile(1);
					break;
			}

			m_mode = newMode;
		}
	}

	@Override
	protected synchronized void readPeriodicInputs(final double timestamp) {
		m_periodicIO.isBusy = m_mode == Mode.POSITION
			  && m_motor.getVelocityLinearMetersPerSecond() < Units.inches2Meters(1)
			  && MathUtils.epsilonEquals(m_periodicIO.referenceMeters, m_periodicIO.positionMeters, Units.inches2Meters(1.5));
		m_periodicIO.selectedProfile = m_motor.getSelectedProfile();
		m_periodicIO.ballObserved = m_switch.get();
		m_periodicIO.positionMeters = m_motor.getPositionMeters();
		m_periodicIO.velocityMetersPerSecond = m_motor.getVelocityLinearMetersPerSecond();
	}

	@Override
	protected synchronized void onCreate(final double timestamp) {
		setHoldPosition();
	}

	@Override
	protected synchronized void onUpdate(final double timestamp) {
		if (m_mode == Mode.VELOCITY) {
			m_motor.setVelocityMetersPerSecond(m_periodicIO.referenceMeters);
		} else if (m_mode == Mode.POSITION) {
			m_motor.setPositionMeters(m_periodicIO.referenceMeters);
		}
	}

	@Override
	protected synchronized void onStop(double timestamp) {

	}

	@Override
	public void setSafeState() {

	}

	public synchronized boolean isBallAtBottom() {
		return m_switch.get();
	}

	public synchronized void setVelocity(final double velocityMetersPerSecond) {
		configControlMode(Mode.VELOCITY);
		m_periodicIO.referenceMeters = velocityMetersPerSecond;

		if (velocityMetersPerSecond < 0) {
			m_periodicIO.ballObserved = false;
			m_periodicIO.ballImplied = false;
		}
	}

	private synchronized void addPosition(final double meters) {
		configControlMode(Mode.POSITION);
		m_periodicIO.referenceMeters = m_periodicIO.positionMeters + meters;
	}

	public synchronized void carryBallToTop() {
		addPosition(BOTTOM_TO_TOP_DISTANCE);
		m_periodicIO.ballImplied = true;
	}

	public synchronized void putBallInHopper() {
		addPosition(TOP_TO_HOPPER_DISTANCE);
		m_periodicIO.ballImplied = false;
	}

	public synchronized void setHoldPosition() {
		addPosition(0.0);
	}

	public synchronized boolean isBusy() {
		return m_periodicIO.isBusy;
	}

	public synchronized boolean hasBall() {
		return m_periodicIO.ballObserved || m_periodicIO.ballImplied;
	}

	public synchronized boolean isBallAtTop() {
		return m_periodicIO.ballImplied && !m_periodicIO.isBusy;
	}

	public Inventory.Location getLocation() {
		return m_location;
	}
}
