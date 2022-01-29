package com.gemsrobotics.demo2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.controls.PIDFController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.subsystems.Turret;

import java.util.Objects;

public final class GreyTTurret extends Subsystem implements Turret {
	private static final double REDUCTION = 1.0 / 140.0;
	private static final double ENCODER_COUNTS_PER_REVOLUTION = 2048.0; // unused, integrated sensor
	private static final MotorController.GearingParameters GEARING_PARAMETERS =
			new MotorController.GearingParameters(REDUCTION, 1.0, ENCODER_COUNTS_PER_REVOLUTION);
	private static final PIDFController.Gains GAINS = new PIDFController.Gains(0.0, 0.0, 0.0, 0.0);
	private static final int MOTOR_PORT = 5;

	private static GreyTTurret INSTANCE;

	public static GreyTTurret getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new GreyTTurret();
		}

		return INSTANCE;
	}


	private final MotorController<TalonFX> m_motor;
	private final PeriodicIO m_periodicIO;

	private Mode m_mode;

	private GreyTTurret() {
		m_motor = MotorControllerFactory.createDefaultTalonFX(MOTOR_PORT);
		m_motor.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		m_motor.setGearingParameters(GEARING_PARAMETERS);
		m_motor.setPIDF(GAINS);
		m_motor.setInvertedOutput(false);
		m_motor.setSelectedProfile(0);

		m_periodicIO = new PeriodicIO();
	}

	public enum Mode {
		DISABLED,
		ROTATION
	}

	private static class PeriodicIO {
		public double currentAmps = 0.0;
		public Rotation reference;
		public Rotation position;
		public Rotation velocity;
	}

	@Override
	protected void readPeriodicInputs(double timestamp) {
		m_periodicIO.currentAmps = m_motor.getDrawnCurrentAmps();
	}

	@Override
	protected void onStart(double timestamp) {

	}

	@Override
	protected void onUpdate(double timestamp) {

	}

	public synchronized void setDisabled() {
		m_mode = Mode.DISABLED;
	}

	@Override
	public void setReference(Rotation reference) {

	}

	@Override
	public boolean atReference() {
		return false;
	}

	@Override
	public Rotation getRotation() {
		return null;
	}

	@Override
	protected void onStop(double timestamp) {

	}

	@Override
	public void setSafeState() {

	}
}
