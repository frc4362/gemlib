package com.gemsrobotics.frc2022.autonomous;

import com.gemsrobotics.frc2022.commands.SetTurretLockedCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LockTurretTestAuton extends SequentialCommandGroup {
	public LockTurretTestAuton() {
		addCommands(
				new WaitCommand(5.0),
				new SetTurretLockedCommand(true),
				new WaitCommand(10.0),
				new SetTurretLockedCommand(false),
				new WaitCommand(5.0)
		);
	}
}
