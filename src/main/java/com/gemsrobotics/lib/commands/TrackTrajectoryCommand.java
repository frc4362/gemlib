package com.gemsrobotics.lib.commands;

import com.gemsrobotics.lib.telemetry.reporting.ReportingEndpoint.Event.Kind;
import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDrive;
import com.gemsrobotics.lib.telemetry.reporting.Reportable;
import com.gemsrobotics.lib.math.se2.RigidTransformWithCurvature;
import com.gemsrobotics.lib.trajectory.Trajectory;
import com.gemsrobotics.lib.trajectory.TrajectoryContainer;
import com.gemsrobotics.lib.trajectory.TrajectoryIterator;
import com.gemsrobotics.lib.trajectory.parameterization.TimedState;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class TrackTrajectoryCommand extends Command implements Reportable {
    protected final DifferentialDrive<?> m_chassis;
    protected final boolean m_usesHighGear;
    protected final TrajectoryIterator<TimedState<RigidTransformWithCurvature>> m_trajectory;

    public TrackTrajectoryCommand(final DifferentialDrive<?> chassis, final TrajectoryContainer<RigidTransformWithCurvature> path) {
        setName("FollowTrajectory");
        requires(chassis);

        m_chassis = chassis;
        m_usesHighGear = path.usesHighGear();
        m_trajectory = path.getTrajectory();
    }

    public TrackTrajectoryCommand(final DifferentialDrive<?> chassis, final Trajectory<TimedState<RigidTransformWithCurvature>> trajectory) {
        this(chassis, new TrajectoryContainer<>(false, trajectory));
    }

    @Override
    public void initialize() {
        m_chassis.setHighGear(m_usesHighGear);
        m_chassis.setTrajectory(m_trajectory);
    }

    @Override
    public boolean isFinished() {
        return m_chassis.isTrajectoryFinished();
    }

    @Override
    public void end() {
        report("Finished.");
        report(Kind.INQUIRY, "Did it work?");
        m_chassis.setDisabled();
    }

    @Override
    public void interrupted() {
        end();
    }

    public Command onProgressRemaining(final double progressSeconds, final Command command) {
        return new CommandGroup() {
            {
                addSequential(new Command() {
                    @Override
                    public boolean isFinished() {
                        return m_trajectory.getRemainingProgress() < progressSeconds;
                    }
                });
                addSequential(command);
            }
        };
    }
}
