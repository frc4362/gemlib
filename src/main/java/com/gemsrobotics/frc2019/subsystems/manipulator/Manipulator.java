package com.gemsrobotics.frc2019.subsystems.manipulator;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;

@SuppressWarnings("unused")
public class Manipulator {
	private static final CANSparkMaxLowLevel.ConfigParameter SENSOR_PARAMETER =
			CANSparkMaxLowLevel.ConfigParameter.kSensorType;
	private static final int NO_SENSOR_SETTING_ID = 0;

	private final WPI_TalonSRX m_stage1;
	private final CANSparkMax m_stage2Master, m_stage2Slave;
	private final Solenoid m_arm, m_hand;

	private RunMode m_intakeSetpoint;

	public Manipulator(final ManipulatorConfig config) {
		m_stage2Master = new CANSparkMax(config.stage2Port[0], MotorType.kBrushed);
		m_stage2Master.setIdleMode(CANSparkMax.IdleMode.kBrake);
		disableEncoder(m_stage2Master);

		m_stage2Slave = new CANSparkMax(config.stage2Port[1], MotorType.kBrushed);
		m_stage2Slave.setIdleMode(CANSparkMax.IdleMode.kBrake);
		m_stage2Slave.follow(m_stage2Master, true);
		disableEncoder(m_stage2Slave);

		m_stage1 = new WPI_TalonSRX(config.stage1Port);

		m_arm = new Solenoid(config.placementPort);
		m_hand = new Solenoid(config.longPlacePort);

		m_intakeSetpoint = RunMode.NEUTRAL;
	}

	private void disableEncoder(final CANSparkMax motor) {
		motor.setParameter(SENSOR_PARAMETER, NO_SENSOR_SETTING_ID);
	}

	public enum RunMode {
		INTAKING(-1.0, -0.7),
		INTAKING_RAISED(-1.0, 0.0),
		EXHAUSTING(0.45, 0.7),
		EXHAUSTING_RAISED(0.45, 0.0),
		NEUTRAL(-0.25, 0.0),
		HALTED(0.0, 0.0);

		private final double speed2, speed1;

		RunMode(final double s, final double s1) {
			speed2 = s;
			speed1 = s1;
		}
	}

	public enum PlacementState {
		HOLDING, SHORT_PLACE, LONG_PICKUP, LONG_PLACED
	}

	private void setSpeed(final double speed2, final double speed1) {
		m_stage1.set(speed1);
		m_stage2Master.set(speed2);
	}

	public void setSetSpeed(final RunMode mode) {
		m_intakeSetpoint = mode;
		setSpeed(mode.speed2, mode.speed1);
	}

	public void set(final PlacementState state) {
		switch (state) {
			case SHORT_PLACE:
				m_arm.set(false);
				m_hand.set(true);
				break;

			case LONG_PICKUP:
				m_arm.set(true);
				m_hand.set(true);
				break;

			case LONG_PLACED:
				m_arm.set(false);
				m_hand.set(true);
				break;

			default:
			case HOLDING:
				m_arm.set(false);
				m_hand.set(false);
				break;
		}
	}

	public RunMode getCurrentRunMode() {
		return m_intakeSetpoint;
	}

	public Solenoid getHand() {
		return m_hand;
	}

	public Solenoid getArm() {
		return m_arm;
	}
}
