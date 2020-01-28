package com.gemsrobotics.lib.commands;

import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.physics.Characterizer;
import edu.wpi.first.wpilibj.command.Command;

import java.util.List;

import static java.lang.Math.abs;

public class CollectMotorVelocityData<MotorType> extends Command {    private static final double MAX_POWER = 0.25;
	private static final double RAMP_RATE = 0.02;

	private final MotorController<MotorType> m_motor;
	private final List<Characterizer.VelocityDataPoint> m_data;

	private double m_power;

	/**
	 * @param chassis  the drive base to characterize
	 * @param data     reference to the list where data points should be stored
	 */
	public CollectMotorVelocityData(final MotorController<MotorType> chassis, final List<Characterizer.VelocityDataPoint> data) {
		m_motor = chassis;
		m_data = data;
	}

	@Override
	public void initialize() {
		m_power = 0.0;
	}

	@Override
	public void execute() {
		m_power = RAMP_RATE * timeSinceInitialized();
		m_motor.setDutyCycle(m_power);

		final var data = new Characterizer.VelocityDataPoint(
				abs(m_motor.getVelocityAngularRadiansPerSecond()),
				m_power * m_motor.getVoltageInput());

		m_data.add(data);
	}

	@Override
	public boolean isFinished() {
		return m_power > MAX_POWER;
	}

	@Override
	public void end() {
		m_motor.setDutyCycle(0.0);
	}
}
