package com.gemsrobotics.frc2018.subsystems;

import com.gemsrobotics.lib.FileObjects;
import com.gemsrobotics.lib.drivers.motorcontrol.GemSparkMax;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.drivers.transmission.DualSpeedTransmission;
import com.gemsrobotics.lib.drivers.transmission.Transmission;
import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDrive;
import edu.wpi.first.wpilibj.Solenoid;

import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public final class Chassis extends DifferentialDrive {
	private static Chassis INSTANCE;

	public static Chassis getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Chassis();
		}

		return INSTANCE;
	}

    @Override
    protected Config getConfig() {
        return FileObjects.get(DifferentialDrive.Config.class, "drivetrain_properties");
    }

    @Override
    protected List<MotorController> getMotorControllers() {
        return Stream.of(1, 2, 3, 4).map(MotorControllerFactory::createDefaultTalonSRX).collect(Collectors.toList());
    }

    @Override
    protected Transmission getTransmission() {
        final var transmission = new DualSpeedTransmission(3);
        transmission.setInverted(true);
        return transmission;
    }
}
