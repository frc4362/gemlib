package com.gemsrobotics.frc2022;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.frc2022.subsystems.*;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.structure.SingleThreadedSubsystemManager;
import com.gemsrobotics.lib.subsystems.Flywheel;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

import static com.gemsrobotics.lib.utils.MathUtils.powSign;
import static java.lang.Math.abs;

public final class Demobot extends TimedRobot {
	private static final double kPeriod = 0.01;

	public static final String DASHBOARD_KEY_TURRET_POSITION = "Turret Position (rotations)";
	public static final int FORWARD_SENSOR_LIMIT = 198300;
	private Chassis m_chassis;
	private Intake m_intake;
	private Uptake m_uptake;
	private Flywheel m_shooterLower, m_shooterUpper;
	private Superstructure m_superstructure;
	private SingleThreadedSubsystemManager m_subsystemManager;
	private XboxController m_gamepad;

	private TalonFX a, b;

//	private GreyTTurret m_greytestTurret;
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
//		m_greytestTurret = GreyTTurret.getInstance();

		m_superstructure = Superstructure.getInstance();
		m_subsystemManager = new SingleThreadedSubsystemManager(List.of(
				m_chassis,
				m_intake,
				m_uptake,
				m_shooterLower,
				m_shooterUpper,
				m_superstructure
		));

		m_chassis.getOdometer().reset(Timer.getFPGATimestamp(), RigidTransform.identity());
		m_chassis.setHeading(Rotation.degrees(0));

		SmartDashboard.setDefaultNumber(DASHBOARD_KEY_TURRET_POSITION, 0.0);

		a = new TalonFX(10);
		a.setNeutralMode(NeutralMode.Brake);
		a.setInverted(true);
		a.configForwardSoftLimitEnable(true);
		a.configForwardSoftLimitThreshold(FORWARD_SENSOR_LIMIT);
		a.configReverseSoftLimitEnable(true);
		a.configReverseSoftLimitThreshold(0);
		a.overrideSoftLimitsEnable(true);
		b = new TalonFX(11);
		b.setNeutralMode(NeutralMode.Brake);
		b.setInverted(false);
		b.configForwardSoftLimitEnable(true);
		b.configForwardSoftLimitThreshold(FORWARD_SENSOR_LIMIT);
		b.configReverseSoftLimitEnable(true);
		b.configReverseSoftLimitThreshold(0);
		b.overrideSoftLimitsEnable(true);
	}

	@Override
	public void disabledInit() {
		m_subsystemManager.stop();
	}

	@Override
	public void teleopInit() {
		m_subsystemManager.start();
		PneumaticsContainer.getInstance().getSwingSolenoid().set(DoubleSolenoid.Value.kForward);
	}

	@Override
	public void teleopPeriodic() {
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
		} else if (m_gamepad.getYButton()) {
			m_superstructure.setWantedState(Superstructure.WantedState.OUTTAKING);
		} else if (m_gamepad.getLeftBumper()) {
			m_superstructure.setWantedState(Superstructure.WantedState.SHOOTING);
		} else {
			m_superstructure.setWantedState(Superstructure.WantedState.IDLE);
		}

//		final double turretSetpoint = SmartDashboard.getNumber(DASHBOARD_KEY_TURRET_POSITION, 0.0);
//		SmartDashboard.putNumber(DASHBOARD_KEY_TURRET_POSITION + " mimic", turretSetpoint);
//		m_greytestTurret.setReference(Rotation.radians(turretSetpoint * MathUtils.Tau));

		a.set(TalonFXControlMode.PercentOutput, m_gamepad.getLeftTriggerAxis() - m_gamepad.getRightTriggerAxis());
		b.set(TalonFXControlMode.PercentOutput, m_gamepad.getLeftTriggerAxis() - m_gamepad.getRightTriggerAxis());
		SmartDashboard.putNumber("lift height a", a.getSelectedSensorPosition());
		SmartDashboard.putNumber("lift height b", b.getSelectedSensorPosition());

		if (m_gamepad.getBButtonPressed()) {
			PneumaticsContainer.getInstance().getSwingSolenoid().toggle();
		}

		m_subsystemManager.update();
	}
}
