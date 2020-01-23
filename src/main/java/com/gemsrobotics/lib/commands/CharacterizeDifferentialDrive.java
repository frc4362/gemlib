package com.gemsrobotics.lib.commands;

import com.gemsrobotics.lib.controls.MotorFeedforward;
import com.gemsrobotics.lib.physics.Characterizer;
import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDrive;
import com.gemsrobotics.lib.telemetry.reporting.Reportable;
import com.gemsrobotics.lib.telemetry.reporting.ReportingEndpoint.Event.Kind;
import com.gemsrobotics.lib.utils.FastDoubleToString;
import edu.wpi.first.wpilibj.command.CommandGroup;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public final class CharacterizeDifferentialDrive extends CommandGroup implements Reportable {
    private final List<Characterizer.VelocityDataPoint> m_velocities;
    private final List<Characterizer.AccelerationDataPoint> m_accelerations;

    private MotorFeedforward.Constants m_output;

    public CharacterizeDifferentialDrive(final DifferentialDrive chassis, final boolean useHighGear) {
        m_output = null;
        m_velocities = new ArrayList<>();
        m_accelerations = new ArrayList<>();

        addSequential(new SetDriveGearCommand(chassis, useHighGear));
        addSequential(new WaitCommand(2.0));
        addSequential(new CollectDriveVelocityData(chassis, m_velocities));
        addSequential(new WaitCommand(2.0));
        addSequential(new CollectDriveAccelerationData(chassis, m_accelerations));
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

        System.out.println("Characterization complete: " + "kS: " + FastDoubleToString.format(constants.kStiction) + ", "
                + "kV: " + FastDoubleToString.format(constants.kV) +
                "kA: " + FastDoubleToString.format(constants.kA));
    }
}
