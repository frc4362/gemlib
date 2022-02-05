package com.gemsrobotics.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.controls.PIDFController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.structure.Subsystem;
import edu.wpi.first.math.controller.ElevatorFeedforward;

import java.util.Objects;

public final class ClimbElevator extends Subsystem {
	private static final int
			ELEVATOR_MASTER_PORT = 8,
			ELEVATOR_SLAVE_PORT = 9;
	private static final double
			SPROCKET_RADIUS = 1.0,
			ELEVATOR_HEIGHT_METERS = 1.0;
	private static final MotorController.GearingParameters
			GEARING_PARAMETERS = new MotorController.GearingParameters(1.0, SPROCKET_RADIUS, 2048);
	private static final PIDFController.Gains ELEVATOR_GAINS =
			new PIDFController.Gains(0.0, 0.0, 0.0, 0.0);
	private static final ElevatorFeedforward ELEVATOR_FEEDFORWARD =
			new ElevatorFeedforward(0.0, 0.0, 0.0, 0.0);
	private static final MotorController.MotionParameters ELEVATOR_MOTION_PARAMETERS =
			new MotorController.MotionParameters(1.0, 1.0, 1.0);
	private static final StatorCurrentLimitConfiguration ELEVATOR_CURRENT_LIMIT =
			new StatorCurrentLimitConfiguration(false, 100, 100, 1.0);

	private static ClimbElevator INSTANCE;

	public static ClimbElevator getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new ClimbElevator();
		}

		return INSTANCE;
	}

	private final MotorController<TalonFX>
			m_motorMaster,
			m_motorSlave;
	private final PeriodicIO m_periodicIO;
	private Mode m_mode;

	private ClimbElevator() {
		m_motorMaster = MotorControllerFactory.createDefaultTalonFX(ELEVATOR_MASTER_PORT);
		m_motorMaster.setGearingParameters(GEARING_PARAMETERS);
		m_motorMaster.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		m_motorMaster.setInvertedOutput(false);
		m_motorMaster.setSelectedProfile(0);
		m_motorMaster.setMotionParametersLinear(ELEVATOR_MOTION_PARAMETERS);
		m_motorMaster.setPIDF(ELEVATOR_GAINS);
		m_motorMaster.getInternalController().configStatorCurrentLimit(ELEVATOR_CURRENT_LIMIT);

		m_motorSlave = MotorControllerFactory.createDefaultTalonFX(ELEVATOR_SLAVE_PORT);
		m_motorSlave.getInternalController().configStatorCurrentLimit(ELEVATOR_CURRENT_LIMIT);
		m_motorSlave.follow(m_motorMaster, true);

		m_periodicIO = new PeriodicIO();
	}

	private static class PeriodicIO {
		public double positionMeters;
		public double referenceMeters;
		public double currentDrawnAmps;
	}

	private enum Mode {
		DISABLED,
		POSITION
	}

	@Override
	protected synchronized void readPeriodicInputs(final double timestamp) {
		m_periodicIO.positionMeters = m_motorMaster.getPositionMeters();
		m_periodicIO.currentDrawnAmps = m_motorMaster.getDrawnCurrentAmps() + m_motorSlave.getDrawnCurrentAmps();
	}

	@Override
	protected synchronized void onStart(final double timestamp) {
		setDisabled();
	}

	@Override
	protected synchronized void onUpdate(final double timestamp) {
		switch (m_mode) {
			case DISABLED:
				m_motorMaster.setNeutral();
				break;
			case POSITION:
				m_motorMaster.setPositionMeters(m_periodicIO.positionMeters);
				break;
		}
	}

	@Override
	protected synchronized void onStop(final double timestamp) {

	}

	@Override
	public void setSafeState() {

	}

	public synchronized void setReferencePosition(final double referenceMeters) {
		m_mode = Mode.POSITION;
		m_periodicIO.referenceMeters = referenceMeters;
	}

	public synchronized void setDisabled() {
		m_mode = Mode.DISABLED;
	}
}
