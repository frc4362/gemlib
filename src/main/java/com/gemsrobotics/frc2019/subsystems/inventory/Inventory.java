package com.gemsrobotics.frc2019.subsystems.inventory;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@SuppressWarnings("unused")
public abstract class Inventory {
	public enum GamePiece {
		PANEL,
		CARGO,
		NONE
	}

	public abstract boolean hasPanel();
	public abstract boolean hasCargo();

	public final boolean isEmpty() {
		return !hasCargo() && !hasPanel();
	}

	public final GamePiece getCurrentPiece() {
		if (hasPanel()) {
			return GamePiece.PANEL;
		} else if (hasCargo()) {
			return GamePiece.CARGO;
		} else {
			return GamePiece.NONE;
		}
	}

	private static class InventoryLogger extends Command {
		private final Inventory m_inventory;

		@SuppressWarnings("WeakerAccess")
		public InventoryLogger(final Inventory inventory) {
			m_inventory = inventory;
		}

		@Override
		public void execute() {
			SmartDashboard.putString("Current Game Piece", m_inventory.getCurrentPiece().toString());
			SmartDashboard.putBoolean("Cargo Raw Reading", m_inventory.hasCargo());
		}

		@Override
		public boolean isFinished() {
			return false;
		}
	}

	public Command makeLogger() {
		return new InventoryLogger(this);
	}
}
