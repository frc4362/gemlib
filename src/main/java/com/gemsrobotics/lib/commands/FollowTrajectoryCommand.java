package com.gemsrobotics.lib.commands;

import com.gemsrobotics.lib.telemetry.reporting.Reporter;
import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDrive;
import com.gemsrobotics.lib.telemetry.reporting.Reportable;
import com.gemsrobotics.lib.math.se2.RigidTransformWithCurvature;
import com.gemsrobotics.lib.trajectory.TimedView;
import com.gemsrobotics.lib.trajectory.Trajectory;
import com.gemsrobotics.lib.trajectory.TrajectoryContainer;
import com.gemsrobotics.lib.trajectory.TrajectoryIterator;
import com.gemsrobotics.lib.trajectory.parameterization.TimedState;
import edu.wpi.first.wpilibj.command.Command;

public class FollowTrajectoryCommand extends Command implements Reportable {
    protected final DifferentialDrive m_chassis;
    protected final boolean m_usesHighGear;
    protected final Trajectory<TimedState<RigidTransformWithCurvature>> m_trajectory;

    public FollowTrajectoryCommand(
            final DifferentialDrive chassis,
            final TrajectoryContainer path
    ) {
        setName("FollowTrajectory");
        requires(chassis);

        m_chassis = chassis;
        m_usesHighGear = path.usesHighGear;
        m_trajectory = path.trajectory;
    }

    @Override
    public void initialize() {
        m_chassis.setHighGear(m_usesHighGear);
        m_chassis.setTrajectory(new TrajectoryIterator<>(new TimedView<>(m_trajectory)));
    }

    @Override
    public boolean isFinished() {
        return m_chassis.isTrajectoryFinished();
    }

    @Override
    public void end() {
        report("Finished.");
        report(Reporter.Event.Kind.INQUIRY, "Did it work?");
    }

    @Override
    public void interrupted() {
        end();
    }
}
