package com.gemsrobotics.frc2019.commands;

import com.gemsrobotics.frc2019.subsystems.drive.DifferentialDrive;
import com.gemsrobotics.frc2019.util.DualTransmission;
import com.gemsrobotics.frc2019.util.command.ToggleableCommand;

import static com.gemsrobotics.frc2019.util.DualTransmission.Gear.LOW;
import static com.gemsrobotics.frc2019.util.DualTransmission.Gear.HIGH;
import static java.lang.Math.abs;

public class ShiftScheduler extends ToggleableCommand {
	private static final double PEAK_RPM = 5400;

	private final DifferentialDrive m_driveTrain;
	private final DualTransmission m_transmission;

	private long m_lastShiftTime;

	public ShiftScheduler(final DifferentialDrive driveTrain) {
		m_driveTrain = driveTrain;
		m_transmission = driveTrain.getTransmission();
	}

	@Override
	public void initialize() {
		m_lastShiftTime = System.currentTimeMillis();
	}

	@Override
	public void whenEnabled() {
		final var currentGear = m_transmission.get();
		final var shouldShiftHigh = isOverSpeed(.90 * PEAK_RPM);// && Hardware.getInstance().getLimelight().getArea() < 1;
		final var shouldShiftLow = !isOverSpeed(.80 * PEAK_RPM);

		if (currentGear == LOW && shouldShiftHigh && timeSinceShift() > 250) {
			m_transmission.set(HIGH);
		} else if (currentGear == HIGH && shouldShiftLow && timeSinceShift() > 250) {
			m_transmission.set(LOW);
		}
	}

	private double timeSinceShift() {
		return System.currentTimeMillis() - m_lastShiftTime;
	}

	private boolean isOverSpeed(final double targetSpeed) {
		final var left = m_driveTrain.getMotor(DifferentialDrive.Side.LEFT).getEncoder().getVelocity();
		final var right = m_driveTrain.getMotor(DifferentialDrive.Side.RIGHT).getEncoder().getVelocity();
		return abs(left) > targetSpeed && abs(right) > targetSpeed;
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
