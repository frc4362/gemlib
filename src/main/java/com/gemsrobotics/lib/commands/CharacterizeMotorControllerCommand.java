package com.gemsrobotics.lib.commands;

import com.gemsrobotics.lib.controls.MotorFeedforward;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.physics.Characterizer;
import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDrive;
import com.gemsrobotics.lib.utils.FastDoubleToString;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class CharacterizeMotorControllerCommand<MotorType> extends CommandGroup {
	private final List<Characterizer.VelocityDataPoint> m_velocities;
	private final List<Characterizer.AccelerationDataPoint> m_accelerations;

	private MotorFeedforward.Constants m_output;

	public CharacterizeMotorControllerCommand(final MotorController<MotorType> motor) {
		m_output = null;
		m_velocities = new ArrayList<>();
		m_accelerations = new ArrayList<>();

		addSequential(new CollectMotorVelocityData<>(motor, m_velocities));
		addSequential(new WaitCommand(2.0));
		addSequential(new CollectMotorAccelerationData<>(motor, m_accelerations));
	}

	public Optional<MotorFeedforward.Constants> getCharacterizationConstants() {
		return Optional.ofNullable(m_output);
	}

	@Override
	protected void initialize() {
		m_output = null;
		m_velocities.clear();
		m_accelerations.clear();
	}

	@Override
	protected void end() {
		final MotorFeedforward.Constants constants = Characterizer.generateCharacterization(m_velocities, m_accelerations);
		m_output = constants;

		System.out.println("Characterization complete: " + "kS: " + FastDoubleToString.format(constants.kStiction,6) + ", "
								   + "kV: " + FastDoubleToString.format(constants.kV, 6) +
								   "kA: " + FastDoubleToString.format(constants.kA, 6));
	}
}
