package com.gemsrobotics.frc2019;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.gemsrobotics.frc2019.subsystems.adjuster.LateralAdjuster;
import com.gemsrobotics.frc2019.subsystems.adjuster.LateralAdjusterConfig;
import com.gemsrobotics.frc2019.subsystems.inventory.*;
import com.gemsrobotics.frc2019.subsystems.drive.DifferentialDrive;
import com.gemsrobotics.frc2019.subsystems.drive.DrivePorts;
import com.gemsrobotics.frc2019.subsystems.manipulator.Manipulator;
import com.gemsrobotics.frc2019.subsystems.manipulator.ManipulatorConfig;
import com.gemsrobotics.frc2019.subsystems.lift.LiftConfig;
import com.gemsrobotics.frc2019.subsystems.lift.Lift;
import com.gemsrobotics.frc2019.subsystems.pto.PTO;
import com.gemsrobotics.frc2019.subsystems.pto.PTOConfig;
import com.gemsrobotics.frc2019.util.MyAHRS;
import com.gemsrobotics.frc2019.util.camera.Limelight;
import edu.wpi.first.wpilibj.*;
import com.moandjiezana.toml.Toml;

import java.util.Objects;

import static com.gemsrobotics.frc2019.Config.getConfig;

@SuppressWarnings({"unused", "WeakerAccess"})
public class Hardware {
	private final LateralAdjuster m_lateral;
	private final DifferentialDrive m_chassis;
	private final Lift m_lift;
	private final Compressor m_compressor;
	private final Manipulator m_manipulator;
	private final Limelight m_limelight;
	private final Inventory m_inventory;
	private final MyAHRS m_ahrs;
	private final Relay m_leds;
	private final PTO m_pto;
	private final DoubleSolenoid m_legsBack;
	private final Solenoid m_legsFront;
	private final WPI_TalonSRX m_rollers;

	private static Hardware INSTANCE;

	public static Hardware getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Hardware();
		}

		return INSTANCE;
	}

	protected Hardware() {
		final Toml
				driveCfg = getConfig("drive"),
				shifterCfg = getConfig("shifter"),
				liftCfg = getConfig("lift"),
				manipulatorCfgRaw = getConfig("manipulator"),
				inventoryCfg = getConfig("inventory"),
				ledsCfg = getConfig("relay"),
				lateralCfg = getConfig("lateralAdjuster"),
				ptoCfg = getConfig("pto");

		final var reflectiveCfg = inventoryCfg.to(ReflectiveInventoryConfig.class);
		m_inventory = new ReflectiveInventory(reflectiveCfg.port);
		m_leds = new Relay(ledsCfg.getLong("port").intValue());
		m_limelight = new Limelight();
		m_compressor = new Compressor();
		m_lift = new Lift(liftCfg.to(LiftConfig.class));
		m_ahrs = new MyAHRS(SPI.Port.kMXP);

		final var manipulatorConfig = manipulatorCfgRaw.to(ManipulatorConfig.class);
		m_manipulator = new Manipulator(manipulatorConfig);
		m_legsFront = new Solenoid(manipulatorConfig.extenderPort);
		m_legsBack = new DoubleSolenoid(6, 7);
		m_pto = new PTO(ptoCfg.to(PTOConfig.class));
		m_rollers = new WPI_TalonSRX(7);
		m_lateral = new LateralAdjuster(lateralCfg.to(LateralAdjusterConfig.class));

		final var shifter = new Solenoid(shifterCfg.getLong("port").intValue());
		m_chassis = new DifferentialDrive(
				driveCfg.getTable("ports").to(DrivePorts.class),
				driveCfg.getTable("localizations").to(DifferentialDrive.Specifications.class),
				shifter,
				m_ahrs,
				false
		);
	}

	public DifferentialDrive getChassis() {
		return m_chassis;
	}

	public Lift getLift() {
		return m_lift;
	}

	public Compressor getCompressor() {
		return m_compressor;
	}

	public Manipulator getManipulator() {
		return m_manipulator;
	}

	public Limelight getLimelight() {
		return m_limelight;
	}

	public Inventory getInventory() {
		return m_inventory;
	}

	public MyAHRS getAHRS() {
		return m_ahrs;
	}

	public Relay getLEDs() {
		return m_leds;
	}

	public LateralAdjuster getLateralAdjuster() {
		return m_lateral;
	}

	public PTO getPTO() {
		return m_pto;
	}

	public DoubleSolenoid getBackLegs() {
		return m_legsBack;
	}

	public Solenoid getStage1Solenoid() {
		return m_legsFront;
	}

	public WPI_TalonSRX getRollers() {
		return m_rollers;
	}
}
