package com.gemsrobotics.frc2022.subsystems;

import com.ctre.phoenix.CANifier;
import com.gemsrobotics.lib.drivers.leds.LEDCanifier;

import java.util.Objects;

public final class LEDs extends LEDCanifier {
	private static LEDs INSTANCE;

	public static LEDs getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new LEDs();
		}

		return INSTANCE;
	}

	private LEDs() {
		super(new CANifier(60));
	}
}
