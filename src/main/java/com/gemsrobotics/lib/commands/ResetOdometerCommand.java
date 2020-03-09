package com.gemsrobotics.lib.commands;

import com.gemsrobotics.lib.math.se2.RigidTransformWithCurvature;
import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDrive;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class ResetOdometerCommand extends InstantCommand {
    private final DifferentialDrive<?> m_chassis;
    private final RigidTransform m_newPose;

    public ResetOdometerCommand(final DifferentialDrive<?> chassis, final RigidTransform startingPose) {
        m_chassis = chassis;
        m_newPose = startingPose;
    }

    public ResetOdometerCommand(final DifferentialDrive<?> chassis, final Trajectory<RigidTransformWithCurvature> trajectory) {
        this(chassis, trajectory.getFirstState().getRigidTransform());
    }

    public ResetOdometerCommand(final DifferentialDrive<?> chassis) {
        this(chassis, RigidTransform.identity());
    }

    @Override
    protected void initialize() {
        m_chassis.getOdometer().reset(Timer.getFPGATimestamp(), m_newPose);
        m_chassis.setHeading(m_newPose.getRotation());
    }
}
