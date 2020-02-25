package com.gemsrobotics.frc2020;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.gemsrobotics.frc2020.subsystems.Hopper;
import com.gemsrobotics.frc2020.subsystems.Shooter;
import com.gemsrobotics.frc2020.subsystems.Turret;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.structure.SubsystemManager;
import com.gemsrobotics.lib.subsystems.Limelight;
import com.gemsrobotics.lib.utils.MathUtils;
import com.gemsrobotics.lib.utils.Units;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;

public class ShooterJig extends TimedRobot implements Loggable {
	@Log
	private Shooter m_shooter;
//	private Turret m_turret;
	private Hopper m_hopper;
//	private Limelight m_limelight;
	private SubsystemManager m_subsystems;
	private XboxController m_gamepad;

	@Override
	public void robotInit() {
//		m_turret = Turret.getInstance();
		m_shooter = Shooter.getInstance();
		m_hopper = Hopper.getInstance();
//		m_limelight = new Limelight() {
//			@Override
//			protected void onStart(double timestamp) {
//				setLEDMode(LEDMode.ON);
//				setSelectedPipeline(0);
//			}
//
//			@Override
//			protected void onUpdate(double timestamp) {
//
//			}
//
//			@Override
//			protected void onStop(double timestamp) {
//
//			}
//		};

		m_subsystems = new SubsystemManager(m_shooter, m_hopper);

		SmartDashboard.putNumber("Shooter RPM", 0.0);
//		SmartDashboard.putNumber("Hopper Duty Cycle", 0.0);
//		SmartDashboard.putNumber("Turret Degrees", 0.0);
		Logger.configureLoggingAndConfig(this, false);

		m_gamepad = new XboxController(2);
	}

	@Override
	public void robotPeriodic() {
//		SmartDashboard.putString("Limelight Horizontal", m_limelight.getOffsetHorizontal().toString());
		Logger.updateEntries();
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void teleopInit() {
		m_subsystems.start();
	}

	@Override
	public void teleopPeriodic() {
//		m_turret.set(ControlMode.PercentOutput, 0.1);
//		m_turret.setReferencePosition(Rotation.degrees(SmartDashboard.getNumber("Turret Degrees", 0.0)));
		final double speed = SmartDashboard.getNumber("Shooter RPM", 0.0);
		m_shooter.setRPM(speed);

		if (m_gamepad.getBumperPressed(GenericHID.Hand.kLeft)) {
			m_hopper.rotate(1);
		} else if (m_gamepad.getBumperPressed(GenericHID.Hand.kRight)) {
			m_hopper.rotate(6);
		}
	}
}
