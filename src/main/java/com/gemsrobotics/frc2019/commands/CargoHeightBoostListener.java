package com.gemsrobotics.frc2019.commands;

import com.gemsrobotics.frc2019.Hardware;
import com.gemsrobotics.frc2019.subsystems.inventory.Inventory;
import com.gemsrobotics.frc2019.subsystems.lift.Lift;
import com.gemsrobotics.frc2019.subsystems.manipulator.Manipulator;
import edu.wpi.first.wpilibj.command.Command;

public class CargoHeightBoostListener extends Command {
	private static final int RAISE_DELAY_MS = 400;

	private final Lift m_lift;
	private final Manipulator m_intake;
	private final Inventory m_inventory;

	private int m_ticksToDelay;
	private boolean m_hadCargoLast;

	public CargoHeightBoostListener(
			final Lift lift,
			final Manipulator intake,
			final Inventory inventory
	) {
		m_lift = lift;
		m_intake = intake;
		m_inventory = inventory;
	}

	@Override
	public void initialize() {
		m_ticksToDelay = -1;
		m_hadCargoLast = false;
	}

	@Override
	public void execute() {
		final boolean hasCargo = m_inventory.hasCargo();

		if (!m_hadCargoLast
			&& hasCargo
			&& (m_intake.getCurrentRunMode() == Manipulator.RunMode.INTAKING)
			&& m_lift.getSetPosition().percent == 0.0
		) {
			m_ticksToDelay = RAISE_DELAY_MS / 20;
		}

		if (m_ticksToDelay == 0) {
			m_lift.setPosition(Lift.Position.CARGO_1);
			Hardware.getInstance().getStage1Solenoid().set(false);
			Hardware.getInstance().getManipulator().setSetSpeed(Manipulator.RunMode.NEUTRAL);
			m_ticksToDelay = -1;
		} else if (m_ticksToDelay > 0) {
			m_ticksToDelay -= 1;
		}

		m_hadCargoLast = hasCargo;
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
