package com.gemsrobotics.demo2022;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.demo2022.subsystems.Chassis;
import com.gemsrobotics.demo2022.subsystems.GreyTTurret;
import com.gemsrobotics.demo2022.subsystems.Shooter;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.structure.SubsystemManager;
import com.gemsrobotics.lib.subsystems.drivetrain.WheelState;
import com.gemsrobotics.lib.utils.MathUtils;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Demobot extends TimedRobot {
	public static final String DASHBOARD_KEY_TURRET_POSITION = "Turret Position (rotations)";
	private Chassis m_chassis;
	private SubsystemManager m_subsystemManager;
	private XboxController m_gamepad;
	private GreyTTurret m_greytestTurret;

	@Override
	public void robotInit() {
		m_gamepad = new XboxController(0);

		m_chassis = Chassis.getInstance();
		m_greytestTurret = GreyTTurret.getInstance();
		m_subsystemManager = new SubsystemManager(m_chassis, m_greytestTurret);

		m_chassis.getOdometer().reset(Timer.getFPGATimestamp(), RigidTransform.identity());
		m_chassis.setHeading(Rotation.degrees(0));

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
		//deadbanding 10%
		if(m_gamepad.getLeftY() > 0.1 || m_gamepad.getLeftY() < -0.1){
			leftY = m_gamepad.getLeftY();
		}
		if(m_gamepad.getRightX() > 0.1 || m_gamepad.getRightX() < -0.1){
			rightX = m_gamepad.getRightX();
		}

		m_chassis.setCurvatureDrive(leftY, rightX, m_gamepad.getRightBumper());
		final double turretSetpoint = SmartDashboard.getNumber(DASHBOARD_KEY_TURRET_POSITION, 0.0);
		SmartDashboard.putNumber(DASHBOARD_KEY_TURRET_POSITION + " mimic", turretSetpoint);
		SmartDashboard.putString("turret reference", m_greytestTurret.getReference().toString());
		SmartDashboard.putString("turret reference rads", Rotation.radians(turretSetpoint * MathUtils.Tau).toString());
		m_greytestTurret.setReference(Rotation.radians(turretSetpoint * MathUtils.Tau));
	}
}
