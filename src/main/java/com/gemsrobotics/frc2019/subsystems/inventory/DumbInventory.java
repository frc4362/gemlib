package com.gemsrobotics.frc2019.subsystems.inventory;

public class DumbInventory extends UltrasonicInventory {
	public DumbInventory(final UltrasonicInventoryConfig cfg) {
		super(cfg);
	}

	@Override
	public boolean hasPanel() {
		return !super.hasCargo();
	}
}
