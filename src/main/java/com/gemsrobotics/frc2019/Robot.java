package com.gemsrobotics.frc2019;

import com.gemsrobotics.frc2019.commands.*;
import com.gemsrobotics.frc2019.util.DualTransmission.Gear;
import com.gemsrobotics.frc2019.util.camera.Limelight.LEDMode;
import com.gemsrobotics.frc2019.util.camera.Limelight.CameraMode;
import com.gemsrobotics.frc2019.util.command.loggers.LimelightLogger;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
	private OperatorInterface m_oi;
	private Hardware m_hardware;
	private SendableChooser<Boolean> m_compressorToggler;

	private boolean m_isFieldMatch;

	@Override
	public void robotInit() {
		m_isFieldMatch = false;
		m_oi = new OperatorInterface(0, 1, 2);

		m_hardware = Hardware.getInstance();

		m_compressorToggler = new SendableChooser<>() {{
			setDefaultOption("Compressor OFF", false);
			addOption("Compressor ON", true);
		}};

		final var driveTrainToggler = new SendableChooser<Boolean>() {{
			setDefaultOption("Drive Train ON", true);
			addOption("Drive Train OFF", false);
		}};

		SmartDashboard.putData("Compressor", m_compressorToggler);
		SmartDashboard.putData("Drive", driveTrainToggler);

		final var limelight = m_hardware.getLimelight();
		limelight.setCameraMode(CameraMode.CV);

		final var chassis = m_hardware.getChassis();
		chassis.getTransmission().set(Gear.LOW);
		chassis.configureDriveCommand(limelight, m_oi, driveTrainToggler);
	}

	@Override
	public void robotPeriodic() {
		SmartDashboard.putBoolean("HIGH GEAR", m_hardware.getChassis().getTransmission().get() == Gear.HIGH);
		m_hardware.getCompressor().setClosedLoopControl(m_compressorToggler.getSelected());

		// TODO
//		m_hardware.getLights().update();
	}

	private void initDriverControl() {
		m_isFieldMatch = m_ds.isFMSAttached();

		m_oi.resetControls();

		m_hardware.getPTO().disengage();
		m_hardware.getBackLegs().set(DoubleSolenoid.Value.kForward);
		m_hardware.getStage1Solenoid().set(false);

		final var controller = m_oi.getController();

		final var limelight = m_hardware.getLimelight();
		limelight.setLEDMode(LEDMode.ON);
		Scheduler.getInstance().add(new LimelightLogger(limelight));

		final var lift = m_hardware.getLift();
		lift.setIdleMode(IdleMode.kBrake);
		Scheduler.getInstance().add(new LiftScrubber(lift, controller));
		Scheduler.getInstance().add(new LiftUnstucker(lift));
		Scheduler.getInstance().add(lift.makeLogger());

		final var chassis = m_hardware.getChassis();
		chassis.getMotors().forEach(motor ->
			motor.setIdleMode(IdleMode.kBrake));
		Scheduler.getInstance().add(chassis.getStateEstimator());
		Scheduler.getInstance().add(chassis.getState().makeLogger());
		Scheduler.getInstance().add(chassis.getDriveCommand());
		Scheduler.getInstance().add(m_hardware.getInventory().makeLogger());
		Scheduler.getInstance().add(new LEDListener(
				m_hardware.getLEDs(),
				m_oi.getController(),
				limelight,
				m_hardware.getInventory()));
		Scheduler.getInstance().add(new VisionAdjuster(
				m_hardware.getLateralAdjuster(),
				limelight,
				m_hardware.getInventory(),
				m_oi.getController(),
				m_hardware.getLift()));
		Scheduler.getInstance().add(new CargoHeightBoostListener(
				lift,
				m_hardware.getManipulator(),
				m_hardware.getInventory()));
	}

	@Override
	public void autonomousInit() {
		Scheduler.getInstance().removeAll();
		initDriverControl();
		m_hardware.getLimelight().setLEDMode(LEDMode.ON);
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		Scheduler.getInstance().removeAll();
		initDriverControl();
		m_hardware.getLimelight().setLEDMode(LEDMode.ON);
		Scheduler.getInstance().add(m_hardware.getChassis().getShiftScheduler());
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
		final var idleMode = m_isFieldMatch ? IdleMode.kBrake : IdleMode.kCoast;
		m_hardware.getChassis().getMotors().forEach(motor ->
			motor.setIdleMode(idleMode));
		m_hardware.getLift().setIdleMode(idleMode);
		m_hardware.getLimelight().setLEDMode(LEDMode.OFF);
	}
}
