package com.gemsrobotics.lib.commands;

import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.physics.Characterizer;
import com.gemsrobotics.lib.subsystems.drivetrain.ChassisState;
import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDrive;
import com.gemsrobotics.lib.subsystems.drivetrain.WheelState;
import com.gemsrobotics.lib.utils.MathUtils;
import edu.wpi.first.wpilibj.command.Command;

import java.util.List;

import static java.lang.Math.abs;

public class CollectMotorAccelerationData<MotorType> extends Command {
	private static final double POWER = 0.5;
	private static final double TOTAL_TIME = 2.0; //how long to run the test for

	private final MotorController<MotorType> m_motor;
	private final List<Characterizer.AccelerationDataPoint> m_data;

	private double m_lastTime, m_lastVelocity;

	/**
	 * @param data     reference to the list where data points should be stored=
	 */
	public CollectMotorAccelerationData(final MotorController<MotorType> motor, final List<Characterizer.AccelerationDataPoint> data) {
		m_motor = motor;
		m_data = data;
	}

	@Override
	public void initialize() {
		m_lastTime = Double.NaN;
		m_lastVelocity = Double.NaN;
		m_motor.setDutyCycle(POWER);
	}

	@Override
	public void execute() {
		final var velocity = m_motor.getVelocityAngularRadiansPerSecond();

		//don't calculate acceleration until we've populated prevTime and prevVelocity
		if (Double.isNaN(m_lastTime)) {
			m_lastTime = timeSinceInitialized();
			m_lastVelocity = velocity;
			return;
		}

		final var now = timeSinceInitialized();
		final double acceleration = (velocity - m_lastVelocity) / (now - m_lastTime);
		final double voltage = m_motor.getVoltageOutput();

		System.out.println("acceleration: " + acceleration);

		//ignore accelerations that are too small
		if (abs(acceleration) > MathUtils.Epsilon) {
			m_data.add(new Characterizer.AccelerationDataPoint(velocity, voltage, acceleration));
		}

		m_lastTime = now;
		m_lastVelocity = velocity;
	}

	@Override
	public boolean isFinished() {
		return timeSinceInitialized() > TOTAL_TIME;
	}

	@Override
	public void end() {
		m_motor.setDutyCycle(0.0);
	}
}
