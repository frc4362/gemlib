package com.gemsrobotics.frc2022.drivers;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.drivers.motorcontrol.GemTalon;

public class CursedClimbGemTalon extends GemTalon<TalonFX> {
	public CursedClimbGemTalon(final TalonFX talon, boolean isSlave) {
		super(talon, isSlave);
	}

	@Override
	protected int getInversionMultiplier() {
		return 1;
	}
}
