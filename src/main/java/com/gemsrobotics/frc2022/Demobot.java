package com.gemsrobotics.frc2022;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.frc2022.subsystems.Chassis;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.structure.SingleThreadedSubsystemManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

import static com.gemsrobotics.lib.utils.MathUtils.powSign;
import static java.lang.Math.abs;

public final class Demobot extends TimedRobot {
	private static final double kPeriod = 0.01;

	public static final String DASHBOARD_KEY_TURRET_POSITION = "Turret Position (rotations)";
	private Chassis m_chassis;
	private SingleThreadedSubsystemManager m_subsystemManager;
	private XboxController m_gamepad;
//	private GreyTTurret m_greytestTurret;

	private MotorController<TalonFX> m_transferMotor;
	private MotorController<TalonFX> m_intakeMotor;

	public Demobot() {
		super(kPeriod);
	}

	@Override
	public void robotInit() {
		m_gamepad = new XboxController(0);

		m_chassis = Chassis.getInstance();
//		m_greytestTurret = GreyTTurret.getInstance();
		m_subsystemManager = new SingleThreadedSubsystemManager(List.of(m_chassis));

		m_chassis.getOdometer().reset(Timer.getFPGATimestamp(), RigidTransform.identity());
		m_chassis.setHeading(Rotation.degrees(0));

		m_transferMotor = MotorControllerFactory.createDefaultTalonFX(6);
		m_transferMotor.setNeutralBehaviour(MotorController.NeutralBehaviour.COAST);
		m_transferMotor.setInvertedOutput(true);

		m_intakeMotor = MotorControllerFactory.createDefaultTalonFX(5);
		m_intakeMotor.setNeutralBehaviour(MotorController.NeutralBehaviour.COAST);
		m_intakeMotor.setInvertedOutput(false);

		SmartDashboard.setDefaultNumber(DASHBOARD_KEY_TURRET_POSITION, 0.0);
	}

	@Override
	public void disabledInit() {
		m_subsystemManager.stop();
	}

	@Override
	public void teleopInit() {
		m_subsystemManager.start();
	}

	@Override
	public void teleopPeriodic() {
		double leftY = 0;
		double rightX = 0;

		//deadbanding 4%
		if (abs(m_gamepad.getLeftY()) > 0.04) {
			leftY = -m_gamepad.getLeftY();
		}
		if (abs(m_gamepad.getRightX()) > 0.10) {
			rightX = m_gamepad.getRightX();
		}

		m_chassis.setCurvatureDrive(powSign(leftY, 2.0), rightX, m_gamepad.getRightBumper());
		SmartDashboard.putString("Linear/Angular", Double.toString(powSign(leftY, 2.0)) + "/" + rightX);

//		final double turretSetpoint = SmartDashboard.getNumber(DASHBOARD_KEY_TURRET_POSITION, 0.0);
//		SmartDashboard.putNumber(DASHBOARD_KEY_TURRET_POSITION + " mimic", turretSetpoint);
//		m_greytestTurret.setReference(Rotation.radians(turretSetpoint * MathUtils.Tau));

//		SmartDashboard.putNumber("time", Timer.getFPGATimestamp());

		final double pow = m_gamepad.getLeftTriggerAxis() - m_gamepad.getRightTriggerAxis();
		m_transferMotor.setDutyCycle(pow);
		m_intakeMotor.setDutyCycle(pow);

		m_subsystemManager.update();
	}
}
