package com.gemsrobotics.frc2022;

import com.gemsrobotics.frc2022.autonomous.FiveBallAutonWithFender;
import com.gemsrobotics.frc2022.autonomous.FiveBallAutonWithSafe;
import com.gemsrobotics.frc2022.autonomous.TwoBallAuton;
import com.gemsrobotics.frc2022.subsystems.*;
import com.gemsrobotics.frc2022.subsystems.uptake.Uptake;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.structure.SingleThreadedSubsystemManager;
import com.gemsrobotics.lib.subsystems.Flywheel;
import com.gemsrobotics.lib.subsystems.Limelight;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.List;
import java.util.Objects;
import java.util.Optional;

import static com.gemsrobotics.frc2022.Constants.SMARTDASHBOARD_SHOOTER_KEY;
import static com.gemsrobotics.lib.utils.MathUtils.powSign;
import static java.lang.Math.abs;

public final class Blackbird extends TimedRobot {
	private static final double kPeriod = 0.02;

	private Chassis m_chassis;
	private Intake m_intake;
	private Uptake m_uptake;
	private Flywheel m_shooterLower, m_shooterUpper;
	private Climber m_climber;
	private Hood m_hood;

	private GreyTTurret m_greytestTurret;
	private TargetServer m_targetServer;
	private FieldState m_fieldState;

	private Superstructure m_superstructure;
	private SingleThreadedSubsystemManager m_subsystemManager;
	private XboxController m_pilot, m_copilot;

	private SendableChooser<Command> m_autonChooser;
	private FiveBallAutonWithFender m_fiveBall;

	private Timer m_brakeTimer;

	public Blackbird() {
		super(kPeriod);
	}

	@Override
	public void robotInit() {
		m_pilot = new XboxController(0);
		m_copilot = new XboxController(1);
		m_brakeTimer = new Timer();

		m_chassis = Chassis.getInstance();
		m_intake = Intake.getInstance();
		m_uptake = Uptake.getInstance();
		m_shooterLower = LowerWheel.getInstance();
		m_shooterUpper = UpperWheel.getInstance();
		m_climber = Climber.getInstance();
		m_greytestTurret = GreyTTurret.getInstance();
		m_fieldState = FieldState.getInstance();
		m_targetServer = TargetServer.getInstance();
		m_hood = Hood.getInstance();

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
				m_shooterUpper,
				m_hood
		));

		m_chassis.getOdometer().reset(Timer.getFPGATimestamp(), RigidTransform.identity());
		m_chassis.setHeading(Rotation.degrees(0));

		m_autonChooser = new SendableChooser<>();
		m_autonChooser.addOption("None", new WaitCommand(1.0));
		m_autonChooser.addOption("2-Ball", new TwoBallAuton());
		m_autonChooser.addOption("5-Ball", new FiveBallAutonWithSafe());
		SmartDashboard.putData(m_autonChooser);

		if (Constants.DO_SHOOTER_TUNING) {
			SmartDashboard.putNumber(Constants.SMARTDASHBOARD_SHOOTER_KEY, 0.0);
			SmartDashboard.putNumber(Constants.SMARTDASHBOARD_HOOD_KEY, Hood.MIN_ANGLE.getDegrees());
		}

		LiveWindow.disableAllTelemetry();
		DataLogManager.start();
	}

	@Override
	public void robotPeriodic() {
		SmartDashboard.putString("Robot Position", m_chassis.getOdometer().getLatestFieldToVehicleValue().toString());
		SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
	}

	@Override
	public void disabledInit() {
		m_subsystemManager.stop();
		m_brakeTimer.reset();
		m_brakeTimer.start();
	}

	@Override
	public void disabledPeriodic() {
		if (m_brakeTimer.hasElapsed(2.0)) {
			m_chassis.setNeutralBehaviour(MotorController.NeutralBehaviour.COAST);
			m_brakeTimer.reset();
			m_brakeTimer.stop();
		}
	}

	@Override
	public void autonomousInit() {
		CommandScheduler.getInstance().cancelAll();
		m_subsystemManager.start();
		m_targetServer.setLEDMode(Limelight.LEDMode.ON);
		PneumaticsContainer.getInstance().getSwingSolenoid().set(DoubleSolenoid.Value.kForward);
		final var autonCommand = Optional.ofNullable(m_autonChooser.getSelected()).orElseGet(TwoBallAuton::new);
		CommandScheduler.getInstance().schedule(autonCommand);
		m_chassis.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
	}

	@Override
	public void autonomousPeriodic() {
		CommandScheduler.getInstance().run();
		m_subsystemManager.update();
	}

	@Override
	public void teleopInit() {
		CommandScheduler.getInstance().cancelAll();
		m_subsystemManager.start();
		m_targetServer.setLEDMode(Limelight.LEDMode.ON);
		PneumaticsContainer.getInstance().getSwingSolenoid().set(DoubleSolenoid.Value.kForward);
		m_superstructure.setPrepareShot(false);
		m_superstructure.setTurretLocked(false);
		m_chassis.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
	}

	@Override
	public void teleopPeriodic() {
		if (m_superstructure.getSystemState() != Superstructure.SystemState.PULL_TO_BAR
			&& m_superstructure.getSystemState() != Superstructure.SystemState.PLACE_ON_BAR
		) {
			double leftY = 0;
			double rightX = 0;

			//deadbanding
			if (abs(m_pilot.getLeftY()) > 0.04) {
				leftY = -m_pilot.getLeftY();
			}

			if (abs(m_pilot.getRightX()) > 0.05) {
				rightX = m_pilot.getRightX();
			}

			m_chassis.setCurvatureDrive(powSign(leftY, 2.0), rightX * Constants.OPEN_LOOP_TURN_SENSITIVITY, m_pilot.getRightBumper());

			if (m_pilot.getAButton()) {
				m_superstructure.setWantedState(Superstructure.WantedState.INTAKING);
			} else if (m_pilot.getYButton()) {
				m_superstructure.setWantedState(Superstructure.WantedState.OUTTAKING);
			} else if (m_pilot.getLeftBumper()) {
				m_superstructure.setWantedState(Superstructure.WantedState.SHOOTING);
			} else if (m_copilot.getBButton() && m_copilot.getYButton()) {
				m_superstructure.setWantedState(Superstructure.WantedState.PRECLIMBING);
			} else if (m_superstructure.getSystemState() == Superstructure.SystemState.PRECLIMB && m_copilot.getBButton() && m_copilot.getAButton()) {
				m_superstructure.setWantedStateClimb(Superstructure.ClimbGoal.TRAVERSE);
			} else if (m_superstructure.getSystemState() == Superstructure.SystemState.PRECLIMB && m_copilot.getBButton() && m_copilot.getXButton()) {
				m_superstructure.setWantedStateClimb(Superstructure.ClimbGoal.HIGH);
			} else {
				m_superstructure.setWantedState(Superstructure.WantedState.IDLE);
			}
		}

		if (m_copilot.getLeftBumper()) {
			m_superstructure.adjustShot(0.1);
		} else if (m_copilot.getRightBumper()) {
			m_superstructure.adjustShot(-0.1);
		}

		if (Constants.DO_SHOOTER_LOGGING) {
			SmartDashboard.putNumber("Current Drawn Lower", m_shooterLower.getCurrentAmps());
			SmartDashboard.putNumber("Current Drawn Upper", m_shooterUpper.getCurrentAmps());

			SmartDashboard.putNumber("Shooter Reference RPM Upper", m_shooterUpper.m_periodicIO.shooterReferenceRPM);
			SmartDashboard.putNumber("Shooter Reference RPM Lower", m_shooterLower.m_periodicIO.shooterReferenceRPM);

			SmartDashboard.putNumber("Shooter Measured RPM Upper", m_shooterUpper.getVelocityRPM());
			SmartDashboard.putNumber("Shooter Measured RPM Lower", m_shooterLower.getVelocityRPM());
		}

		final var distance = m_targetServer.getTargetInfo()
				.map(TargetServer.TargetInfo::getCameraToTarget)
				.map(RigidTransform::getTranslation)
				.map(Translation::norm);
		SmartDashboard.putString("Distance", distance.map(Object::toString).orElse("No target"));
		SmartDashboard.putBoolean("Distance Good?", distance.map(Constants::isRangeOk).orElse(false));

		m_subsystemManager.update();
	}
}
