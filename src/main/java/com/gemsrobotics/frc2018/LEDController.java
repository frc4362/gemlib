package com.gemsrobotics.frc2018;

import com.ctre.phoenix.CANifier;
import com.gemsrobotics.frc2018.subsystems.Chassis;
import com.gemsrobotics.lib.drivers.leds.LEDCanifier;
import com.gemsrobotics.lib.drivers.leds.LEDs;
import com.gemsrobotics.lib.telemetry.Pod;
import com.gemsrobotics.lib.telemetry.reporting.Reportable;

import java.awt.Color;
import java.util.Objects;
import java.util.Optional;

@SuppressWarnings("WeakerAccess")
public final class LEDController implements Reportable {
	public String getName() {
		return "LEDController";
	}

	private static Optional<LEDController> INSTANCE;

	public static Optional<LEDController> getInstance() {
		if (Objects.isNull(INSTANCE)) {
			try {
				INSTANCE = Optional.of(new LEDController());
			} catch (final Exception exception) {
				Pod.catchThrowable(null, exception);
				INSTANCE = Optional.empty();
			}
		}

		return INSTANCE;
	}

	private final LEDs m_leds;

	private LEDController() throws Exception {
		m_leds = new LEDCanifier(new CANifier(Ports.CANIFIER_PORT));
	}

	public void update() {
		try {
			if (Chassis.getInstance().isHighGear()) {
				m_leds.setColor(Color.RED, 1.0f);
			} else {
				m_leds.setColor(Color.BLACK, 1.0f);
			}
		} catch (final Exception exception) {
			INSTANCE = Optional.empty();
		}
	}
}
