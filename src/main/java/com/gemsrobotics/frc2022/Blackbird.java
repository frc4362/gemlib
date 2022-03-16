package com.gemsrobotics.frc2022;

import com.gemsrobotics.frc2022.autonomous.FiveBallAuton;
import com.gemsrobotics.frc2022.autonomous.TwoBallAuton;
import com.gemsrobotics.frc2022.subsystems.*;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.structure.SingleThreadedSubsystemManager;
import com.gemsrobotics.lib.subsystems.Flywheel;
import com.gemsrobotics.lib.subsystems.Limelight;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.List;
import java.util.Optional;

import static com.gemsrobotics.lib.utils.MathUtils.powSign;
import static java.lang.Math.abs;

public final class Blackbird extends TimedRobot {
	private static final double kPeriod = 0.02;

	public static final String DASHBOARD_KEY_TURRET_POSITION = "Turret Position (rotations)";
	private Chassis m_chassis;
	private Intake m_intake;
	private Uptake m_uptake;
	private Flywheel m_shooterLower, m_shooterUpper;
	private Climber m_climber;

	private GreyTTurret m_greytestTurret;
	private TargetServer m_targetServer;
	private FieldState m_fieldState;

	private Superstructure m_superstructure;
	private SingleThreadedSubsystemManager m_subsystemManager;
	private XboxController m_pilot, m_copilot;

	private SendableChooser<Command> m_autonChooser;
	private FiveBallAuton m_fiveBall;

	public Blackbird() {
		super(kPeriod);
	}

	@Override
	public void robotInit() {
		m_pilot = new XboxController(0);
		m_copilot = new XboxController(1);

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

		m_autonChooser = new SendableChooser<>();
		m_autonChooser.addOption("None", new WaitCommand(1.0));
		m_autonChooser.addOption("2-Ball", new TwoBallAuton());
		m_autonChooser.addOption("5-Ball", new FiveBallAuton());
		SmartDashboard.putData(m_autonChooser);
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
		PneumaticsContainer.getInstance().getSwingSolenoid().set(DoubleSolenoid.Value.kForward);
		final var autonCommand = Optional.ofNullable(m_autonChooser.getSelected())
				.orElse(new TwoBallAuton());
		CommandScheduler.getInstance().schedule(autonCommand);
	}

	@Override
	public void autonomousPeriodic() {
		CommandScheduler.getInstance().run();
		m_subsystemManager.update();
	}

	@Override
	public void teleopInit() {
		m_subsystemManager.start();
		m_targetServer.setLEDMode(Limelight.LEDMode.ON);
		PneumaticsContainer.getInstance().getSwingSolenoid().set(DoubleSolenoid.Value.kForward);
		CommandScheduler.getInstance().cancelAll();
		m_superstructure.setPrepareShot(false);
	}

	@Override
	public void teleopPeriodic() {
		if (m_superstructure.getSystemState() != Superstructure.SystemState.GRAB_MED_BAR
			&& m_superstructure.getSystemState() != Superstructure.SystemState.EXTEND_HIGH_BAR
		) {
			double leftY = 0;
			double rightX = 0;

			//deadbanding
			if (abs(m_pilot.getLeftY()) > 0.04) {
				leftY = -m_pilot.getLeftY();
			}

			if (abs(m_pilot.getRightX()) > 0.10) {
				rightX = m_pilot.getRightX();
			}

			m_chassis.setCurvatureDrive(powSign(leftY, 2.0), rightX * 0.65, m_pilot.getRightBumper());

			if (m_pilot.getAButton()) {
				m_superstructure.setWantedState(Superstructure.WantedState.INTAKING);
			} else if (m_pilot.getYButton()) {
				m_superstructure.setWantedState(Superstructure.WantedState.OUTTAKING);
			} else if (m_pilot.getLeftTriggerAxis() > 0.7) {
				m_superstructure.setWantedState(Superstructure.WantedState.LOW_SHOT);
			} else if (m_pilot.getLeftBumper()) {
				m_superstructure.setWantedState(Superstructure.WantedState.SHOOTING);
			} else if (m_copilot.getBButton() && m_copilot.getYButton()) {
				m_superstructure.setWantedState(Superstructure.WantedState.PRECLIMBING);
			} else if (m_copilot.getBButton() && m_copilot.getAButton()) {
				m_superstructure.setWantedStateClimb(Superstructure.ClimbGoal.TRAVERSE);
			} else if (m_copilot.getBButton() && m_copilot.getXButton()) {
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
		SmartDashboard.putNumber("Distance", distance.orElse(0.0));
		SmartDashboard.putBoolean("Distance Good?", distance.map(f -> (f > 1.39 && f < 2.75)).orElse(false));

		m_subsystemManager.update();
	}
}
