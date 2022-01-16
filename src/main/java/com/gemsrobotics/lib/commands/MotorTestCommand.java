package com.gemsrobotics.lib.commands;

import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.command.Command;

import java.util.Map;
import java.util.stream.DoubleStream;

import static com.gemsrobotics.lib.utils.MathUtils.epsilonEquals;

public final class MotorTestCommand extends Command {
	public static class Config {
		public double runTime, waitTime;
		public double dutyCycle;
		public double rpmTarget, rpmEpsilon;
		public double currentTarget, currentEpsilon;
	}

	private final String m_name;
	private final MotorController m_motor;
	private final Config m_config;
	private final DoubleStream.Builder m_rpms, m_currents;

	public MotorTestCommand(final String name, final MotorController motor, final Config config) {
		m_name = name;
		m_motor = motor;
		m_config = config;
		m_rpms = DoubleStream.builder();
		m_currents = DoubleStream.builder();
	}

	public MotorTestCommand(final String name, final MotorControllerGroup motorGroup, final Config config) {
		this(name, motorGroup.getMaster(), config);
	}

	@Override
	public void initialize() {
		m_motor.setDutyCycle(m_config.dutyCycle);
	}

	@Override
	public void execute() {
		if (timeSinceInitialized() > m_config.waitTime) {
			m_rpms.add(m_motor.getVelocityAngularRPM());
			m_currents.add(m_motor.getDrawnCurrent());
		}
	}

	@Override
	public boolean isFinished() {
		return timeSinceInitialized() > (m_config.waitTime + m_config.runTime);
	}

	@Override
	public void end() {
		m_motor.setDutyCycle(0.0);

		final boolean rpmsGood = epsilonEquals(m_rpms.build().average().orElse(Double.NaN), m_config.rpmTarget, m_config.rpmEpsilon);
		final boolean currentsGood = epsilonEquals(m_currents.build().average().orElse(Double.NaN), m_config.currentTarget, m_config.currentEpsilon);
	}
}
