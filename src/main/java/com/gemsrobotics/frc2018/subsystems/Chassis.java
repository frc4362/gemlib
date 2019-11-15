package com.gemsrobotics.frc2018.subsystems;

import com.gemsrobotics.frc2018.Ports;
import com.gemsrobotics.lib.FileObjects;
import com.gemsrobotics.lib.drivers.motorcontrol.GemSparkMax;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerGroup;
import com.gemsrobotics.lib.drivers.transmission.DualSpeedTransmission;
import com.gemsrobotics.lib.drivers.transmission.Transmission;
import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDrive;
import com.gemsrobotics.lib.telemetry.monitoring.ConnectionMonitor;
import edu.wpi.first.wpilibj.Solenoid;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public final class Chassis extends DifferentialDrive {
	private static Chassis INSTANCE;

	public static synchronized Chassis getInstance() {
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
    protected MotorControllerGroup getMotorControllersLeft() {
        return null;
    }

    @Override
    protected MotorControllerGroup getMotorControllersRight() {
        return null;
    }

    @Override
    protected Transmission getTransmission() {
        final var transmission = new DualSpeedTransmission(3);
        transmission.setInverted(true);
        return transmission;
    }

    @Override
    protected void onEnable(double timestamp) {
        setDisabled();
    }

    @Override
    protected void onStop(double timestamp) {
        setNeutralBehaviour(ConnectionMonitor.getInstance().hasConnectedToField()
                ? MotorController.NeutralBehaviour.BRAKE
                : MotorController.NeutralBehaviour.COAST);
    }
}
