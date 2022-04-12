package com.gemsrobotics.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.frc2022.PneumaticsContainer;
import com.gemsrobotics.lib.controls.PIDFController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.utils.MathUtils;
import com.gemsrobotics.lib.utils.Units;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import java.util.Objects;

import static com.gemsrobotics.lib.utils.MathUtils.coerce;

public final class Climber extends Subsystem {
	private static final int
			ELEVATOR_MASTER_PORT = 10,
			ELEVATOR_SLAVE_PORT = 11;
	private static final double FORWARD_SENSOR_LIMIT = 198300;
	private static final double
			SPROCKET_RADIUS = Units.inches2Meters(1.273) / 2.0,
			GEARING_REDUCTION = 1.0 / 15.0,
			ELEVATOR_BASE_HEIGHT_METERS = Units.inches2Meters(40.0),
			ELEVATOR_MAX_HEIGHT_METERS = Units.inches2Meters(66.0),
			ELEVATOR_THROW = ELEVATOR_MAX_HEIGHT_METERS - ELEVATOR_BASE_HEIGHT_METERS,
			ELEVATOR_TICKS_PER_METER = FORWARD_SENSOR_LIMIT / (ELEVATOR_MAX_HEIGHT_METERS - ELEVATOR_BASE_HEIGHT_METERS),
			START_CLIMB_HEIGHT_METERS = Units.inches2Meters(63.0);
	private static final MotorController.GearingParameters
			GEARING_PARAMETERS = new MotorController.GearingParameters(GEARING_REDUCTION, SPROCKET_RADIUS, 2048);
	private static final PIDFController.Gains ELEVATOR_GAINS = new PIDFController.Gains(0.5, 0.0, 0.0 /*45.591*/, 0.0);
	private static final ElevatorFeedforward ELEVATOR_FEEDFORWARD = new ElevatorFeedforward(0.51654, 0.36935, 16.29, 0.40084);
	private static final StatorCurrentLimitConfiguration ELEVATOR_CURRENT_LIMIT =
			new StatorCurrentLimitConfiguration(false, 100, 100, 1.0);

	private static Climber INSTANCE;

	public static Climber getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Climber();
		}

		return INSTANCE;
	}

	private final MotorController<TalonFX>
			m_motorMaster,
			m_motorSlave;
	private final DoubleSolenoid m_swinger;
	private final PeriodicIO m_periodicIO;
	private Mode m_mode;

	private Climber() {
		m_motorMaster = MotorControllerFactory.createCursedTalonFX(ELEVATOR_MASTER_PORT);
		m_motorMaster.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		m_motorMaster.setGearingParameters(GEARING_PARAMETERS);
		m_motorMaster.getInternalController().setSensorPhase(true);
		m_motorMaster.setInvertedOutput(true);
		m_motorMaster.setSelectedProfile(0);
		m_motorMaster.setPIDF(ELEVATOR_GAINS);
		m_motorMaster.getInternalController().configStatorCurrentLimit(ELEVATOR_CURRENT_LIMIT);
		m_motorMaster.getInternalController().configForwardSoftLimitEnable(true);
		m_motorMaster.getInternalController().configForwardSoftLimitThreshold(FORWARD_SENSOR_LIMIT);
		m_motorMaster.getInternalController().configReverseSoftLimitEnable(true);
		m_motorMaster.getInternalController().configReverseSoftLimitThreshold(0);
		m_motorMaster.getInternalController().overrideSoftLimitsEnable(true);
		m_motorMaster.getInternalController().configAllowableClosedloopError(0, 200);

		m_motorSlave = MotorControllerFactory.createSlaveTalonFX(ELEVATOR_SLAVE_PORT);
		m_motorSlave.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		m_motorSlave.setInvertedOutput(false);
		m_motorSlave.getInternalController().configStatorCurrentLimit(ELEVATOR_CURRENT_LIMIT);
		m_motorSlave.getInternalController().configForwardSoftLimitEnable(true);
		m_motorSlave.getInternalController().configForwardSoftLimitThreshold(FORWARD_SENSOR_LIMIT);
		m_motorSlave.getInternalController().configReverseSoftLimitEnable(true);
		m_motorSlave.getInternalController().configReverseSoftLimitThreshold(0);
		m_motorSlave.getInternalController().overrideSoftLimitsEnable(true);

		m_motorSlave.follow(m_motorMaster, true);

		m_swinger = PneumaticsContainer.getInstance().getSwingSolenoid();

		m_periodicIO = new PeriodicIO();

		setVoltageSettings(VoltageSetting.LOW);
	}

	private static class PeriodicIO {
		public double position = 0.0;
		public double referencePosition = 0.0;
		public double currentDrawnAmps = 0.0;
		public boolean swingerExtended = false;
	}

	private enum Mode {
		DISABLED,
		POSITION
	}

	@Override
	protected void readPeriodicInputs(final double timestamp) {
		m_periodicIO.position = m_motorMaster.getPositionMeters();
		m_periodicIO.currentDrawnAmps = m_motorMaster.getDrawnCurrentAmps() + m_motorSlave.getDrawnCurrentAmps();
	}

	@Override
	protected void onStart(final double timestamp) {
		setDisabled();
	}

	@Override
	protected void onUpdate(final double timestamp) {
//		SmartDashboard.putNumber("Volts Applied", m_motorMaster.getVoltageOutput());
//		SmartDashboard.putNumber("Current Drawn", m_motorMaster.getDrawnCurrentAmps());

		m_swinger.set(m_periodicIO.swingerExtended ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
		switch (m_mode) {
			case DISABLED:
				m_motorMaster.setNeutral();
				break;
			case POSITION:
				m_motorMaster.setPositionMeters(m_periodicIO.referencePosition);
				break;
		}
	}

	@Override
	protected void onStop(final double timestamp) {

	}

	@Override
	public void setSafeState() {

	}

	public void setReferencePercent(final double percent) {
		m_mode = Mode.POSITION;
		m_periodicIO.referencePosition = coerce(0.0, percent * ELEVATOR_THROW, ELEVATOR_THROW * .98);
	}

	public void setGroundOffset(final double referenceMeters) {
		m_mode = Mode.POSITION;
		m_periodicIO.referencePosition = MathUtils.coerce(ELEVATOR_BASE_HEIGHT_METERS, referenceMeters, ELEVATOR_MAX_HEIGHT_METERS) - ELEVATOR_BASE_HEIGHT_METERS;
	}

	public boolean atReference() {
		return MathUtils.epsilonEquals(m_periodicIO.position, m_periodicIO.referencePosition, Units.inches2Meters(1.0));
	}

	public enum VoltageSetting {
		LOW(1.0, -1),
		MID(1.0, -1),
		HIGH(1.0, -1),
		END(1.0, -0.35);

		public final double forwardLimit, reverseLimit;

		VoltageSetting(final double f, final double r) {
			forwardLimit = f;
			reverseLimit = r;
		}
	}

	public void setVoltageSettings(final VoltageSetting sf) {
		m_motorMaster.getInternalController().configPeakOutputForward(sf.forwardLimit);
		m_motorMaster.getInternalController().configPeakOutputReverse(sf.reverseLimit);
		m_motorSlave.getInternalController().configPeakOutputForward(sf.forwardLimit);
		m_motorSlave.getInternalController().configPeakOutputReverse(sf.reverseLimit);
	}

	public void setPreclimbHeight() {
		setGroundOffset(START_CLIMB_HEIGHT_METERS);
	}

	public void setSensorsZero() {
		m_motorMaster.setEncoderCounts(0.0);
		m_motorSlave.setEncoderCounts(0.0);
	}

	public void setSwingerExtended(final boolean extended) {
		m_periodicIO.swingerExtended = extended;
	}

	public void setDisabled() {
		m_mode = Mode.DISABLED;
	}
}
