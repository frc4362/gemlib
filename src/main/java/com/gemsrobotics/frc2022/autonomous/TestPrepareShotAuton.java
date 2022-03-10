package com.gemsrobotics.frc2022.autonomous;

import com.gemsrobotics.frc2022.commands.PrepareShotCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TestPrepareShotAuton extends SequentialCommandGroup {
	public TestPrepareShotAuton() {
		addCommands(
				new PrepareShotCommand(true),
				new WaitCommand(30.0),
				new PrepareShotCommand(false)
		);
	}
}
