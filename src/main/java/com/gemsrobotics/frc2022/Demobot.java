package com.gemsrobotics.frc2022;

import com.gemsrobotics.frc2022.autonomous.TestAuton;
import com.gemsrobotics.frc2022.subsystems.*;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.structure.SingleThreadedSubsystemManager;
import com.gemsrobotics.lib.subsystems.Flywheel;
import com.gemsrobotics.lib.subsystems.Limelight;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

import static com.gemsrobotics.lib.utils.MathUtils.Tau;
import static com.gemsrobotics.lib.utils.MathUtils.powSign;
import static java.lang.Math.abs;

public final class Demobot extends TimedRobot {
	private static final double kPeriod = 0.02;

	public static final String DASHBOARD_KEY_TURRET_POSITION = "Turret Position (rotations)";
	private Chassis m_chassis;
	private Intake m_intake;
	private Uptake m_uptake;
	private Flywheel m_shooterLower, m_shooterUpper;
	private Climber m_climber;
	private TargetServer m_targetServer;
	private FieldState m_fieldState;
	private Superstructure m_superstructure;
	private SingleThreadedSubsystemManager m_subsystemManager;
	private XboxController m_gamepad;

	private GreyTTurret m_greytestTurret;

	public Demobot() {
		super(kPeriod);
	}

	@Override
	public void robotInit() {
		m_gamepad = new XboxController(0);

		m_chassis = Chassis.getInstance();
		m_intake = Intake.getInstance();
		m_uptake = Uptake.getInstance();
		m_shooterLower = LowerWheel.getInstance();
		m_shooterUpper = UpperWheel.getInstance();
		m_climber = Climber.getInstance();
		m_greytestTurret = GreyTTurret.getInstance();
		m_fieldState = FieldState.getInstance();
		m_targetServer = TargetServer.getInstance();

		m_superstructure = Superstructure.getInstance();
		m_subsystemManager = new SingleThreadedSubsystemManager(List.of(
				m_targetServer,
				m_chassis,
				m_intake,
				m_uptake,
				m_climber,
				m_greytestTurret,
				m_fieldState,
				m_superstructure,
				m_shooterLower,
				m_shooterUpper
		));

		m_chassis.getOdometer().reset(Timer.getFPGATimestamp(), RigidTransform.identity());
		m_chassis.setHeading(Rotation.degrees(0));

		SmartDashboard.putNumber("Target MPS", 0.0);
	}

	@Override
	public void robotPeriodic() {
		SmartDashboard.putString("Robot Position", m_chassis.getOdometer().getLatestFieldToVehicleValue().toString());
	}

	@Override
	public void disabledInit() {
		m_subsystemManager.stop();
	}

	@Override
	public void autonomousInit() {
		m_subsystemManager.start();
		m_targetServer.setLEDMode(Limelight.LEDMode.ON);
		Scheduler.getInstance().add(new TestAuton());
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		m_subsystemManager.update();
	}

	@Override
	public void teleopInit() {
		m_subsystemManager.start();
		m_targetServer.setLEDMode(Limelight.LEDMode.ON);
		PneumaticsContainer.getInstance().getSwingSolenoid().set(DoubleSolenoid.Value.kForward);
	}

	@Override
	public void teleopPeriodic() {
		// change for test commit

		if (m_superstructure.getSystemState() != Superstructure.SystemState.GRAB_MED_BAR
			&& m_superstructure.getSystemState() != Superstructure.SystemState.EXTEND_HIGH_BAR
		) {
			double leftY = 0;
			double rightX = 0;

			//deadbanding
			if (abs(m_gamepad.getLeftY()) > 0.04) {
				leftY = -m_gamepad.getLeftY();
			}

			if (abs(m_gamepad.getRightX()) > 0.10) {
				rightX = m_gamepad.getRightX();
			}

			m_chassis.setCurvatureDrive(powSign(leftY, 2.0), rightX, m_gamepad.getRightBumper());

			if (m_gamepad.getAButton()) {
				m_superstructure.setWantedState(Superstructure.WantedState.INTAKING);
			} else if (m_gamepad.getBButton()) {
				m_superstructure.setWantedState(Superstructure.WantedState.PRECLIMBING);
			} else if (m_gamepad.getXButton()) {
				m_superstructure.setWantedState(Superstructure.WantedState.CLIMBING);
			} else if (m_gamepad.getYButton()) {
				m_superstructure.setWantedState(Superstructure.WantedState.OUTTAKING);
			} else if (m_gamepad.getLeftBumper()) {
				m_superstructure.setWantedState(Superstructure.WantedState.SHOOTING);
			}  else {
				m_superstructure.setWantedState(Superstructure.WantedState.IDLE);
			}
		}

//		SmartDashboard.putNumber("Current Drawn Lower", m_shooterLower.getCurrentAmps());
//		SmartDashboard.putNumber("Current Drawn Upper", m_shooterUpper.getCurrentAmps());
//
//		SmartDashboard.putNumber("Shooter Reference RPM Upper", m_shooterUpper.m_periodicIO.shooterReferenceRPM);
//		SmartDashboard.putNumber("Shooter Reference RPM Lower", m_shooterLower.m_periodicIO.shooterReferenceRPM);
//
//		SmartDashboard.putNumber("Shooter Measured RPM Upper", m_shooterUpper.getVelocityRPM());
//		SmartDashboard.putNumber("Shooter Measured RPM Lower", m_shooterLower.getVelocityRPM());

//		final double turretSetpoint = SmartDashboard.getNumber(DASHBOARD_KEY_TURRET_POSITION, 0.0);
//		m_greytestTurret.setReference(Rotation.radians(turretSetpoint * Tau));

//		SmartDashboard.putNumber("Volts", m_chassis.getWheelProperty(MotorController::getVoltageOutput).left);
//		SmartDashboard.putNumber("Amps", m_chassis.getWheelProperty(MotorController::getDrawnCurrentAmps).left);
//		SmartDashboard.putNumber("Robot Speed", m_chassis.getWheelProperty(MotorController::getVelocityLinearMetersPerSecond).left);
		SmartDashboard.putBoolean("Distance Good?", m_targetServer.getTargetInfo()
				.map(TargetServer.TargetInfo::getCameraToTarget)
				.map(RigidTransform::getTranslation)
				.map(Translation::norm)
				.map(f -> (f > 1.39 && f < 3.25)).orElse(false));

		m_subsystemManager.update();
	}
}
