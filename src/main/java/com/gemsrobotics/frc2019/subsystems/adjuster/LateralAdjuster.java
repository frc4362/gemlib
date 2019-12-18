package com.gemsrobotics.frc2019.subsystems.adjuster;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.gemsrobotics.frc2019.util.PIDF;

import static java.lang.Math.*;

@SuppressWarnings("unused")
public class LateralAdjuster {
	public final double kLatVolts;
	private final WPI_TalonSRX m_motor;
	private final double m_widthTicks, m_widthInches, m_radiansToEnd;

	private boolean m_isDisabled;

	public LateralAdjuster(final LateralAdjusterConfig cfg) {
		kLatVolts = cfg.nominalVolts;
		m_radiansToEnd = cfg.adjustmentThresholdRadians;

		m_motor = new WPI_TalonSRX(cfg.port);

		final PIDF pids = cfg.pidVars;
		m_motor.config_kP(0, pids.kP,0);
		m_motor.config_kI(0, pids.kI,0);
		m_motor.config_kD(0, pids.kD,0);
		m_motor.config_kF(0, pids.kF,0);

		m_motor.setNeutralMode(NeutralMode.Brake);

		m_motor.configForwardSoftLimitEnable(true, 0);
		m_motor.configReverseSoftLimitEnable(true, 0);
		m_motor.configForwardSoftLimitThreshold(cfg.leftDist, 0);
		m_motor.configReverseSoftLimitThreshold(cfg.rightDist, 0);

		m_widthTicks = cfg.leftDist - cfg.rightDist;
		m_widthInches = cfg.widthInches;

		m_isDisabled = false;
	}

	private static double constrain(
			final double bot,
			final double val,
			final double top
	) {
		return max(bot, min(val, top));
	}

	public void setTicks(double ticks) {
		m_motor.set(ControlMode.Position, constrain(-m_widthTicks / 2, ticks, m_widthTicks / 2));
	}

	public void setPercent(final double percent) {
		setTicks(percent * m_widthTicks - m_widthTicks / 2);
	}

	public void drive(final double speed) {
		m_motor.set(speed);
	}

	public void stop() {
		m_motor.set(ControlMode.PercentOutput, 0);
	}

	public void alignRadians(final double radiansFromCenter) {
		setPercent((radiansFromCenter / m_radiansToEnd + 1) / 2);
	}

	public WPI_TalonSRX getMotor() {
		return m_motor;
	}

	public double getPosition() {
		return m_motor.getSelectedSensorPosition(0);
	}

	public double getWidthTicks() {
		return m_widthTicks;
	}

	public double getAdjustmentThreshold() {
		return m_radiansToEnd;
	}

	public void disable() {
		m_isDisabled = true;
	}

	public boolean isDisabled() {
		return m_isDisabled;
	}

	public double inches2Ticks(final double inches) {
		return (inches / m_widthInches) * m_widthTicks;
	}
}
