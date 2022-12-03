package com.gemsrobotics.frc2022.autonomous;

import java.util.List;

import com.gemsrobotics.frc2022.commands.GemRamseteCommand;
import com.gemsrobotics.frc2022.commands.ResetOdometerCommand;
import com.gemsrobotics.frc2022.commands.ShootAllBalls;
import com.gemsrobotics.frc2022.subsystems.Chassis;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.utils.Units;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OneBallAuton extends SequentialCommandGroup {
    public OneBallAuton() {
        final var chassis = Chassis.getInstance();
        final var traj = chassis.getReversedTrajectory(List.of(
            RigidTransform.identity(),
            RigidTransform.fromTranslation(new Translation(Units.inches2Meters(-54.0), 0.0))
        ));

        addCommands(
            new ResetOdometerCommand(chassis, traj),
            new GemRamseteCommand(traj),
            new ShootAllBalls()
        );
    }
}
