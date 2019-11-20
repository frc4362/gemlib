package com.gemsrobotics.frc2019.subsystems.inventory;

import edu.wpi.first.wpilibj.DriverStation;

public class ManualInventory extends Inventory {
	@Override
	public boolean hasPanel() {
		return !hasCargo();
	}

	@Override
	public boolean hasCargo() {
		return DriverStation.getInstance().getStickButton(2, 3);
	}
}
