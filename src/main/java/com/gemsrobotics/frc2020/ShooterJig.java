package com.gemsrobotics.frc2020;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.gemsrobotics.frc2020.subsystems.Shooter;
import com.gemsrobotics.lib.structure.SubsystemManager;
import com.gemsrobotics.lib.subsystems.Limelight;
import com.revrobotics.Rev2mDistanceSensor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;

public class ShooterJig extends TimedRobot implements Loggable {
	@Log
	private Shooter m_shooter;
	private Limelight m_limelight;
	private Rev2mDistanceSensor m_distanceSensor;
	private SubsystemManager m_subsystems;
	private TalonSRX m_turret;

	@Override
	public void robotInit() {
		m_shooter = Shooter.getInstance();
		m_limelight = new Limelight() {
			@Override
			protected void onCreate(double timestamp) {
				setLEDMode(LEDMode.ON);
				setSelectedPipeline(0);
			}

			@Override
			protected void onUpdate(double timestamp) {

			}

			@Override
			protected void onStop(double timestamp) {

			}
		};

		m_subsystems = new SubsystemManager(m_shooter, m_limelight);

		m_turret = new TalonSRX(39);

		m_distanceSensor = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kOnboard);
		m_distanceSensor.setEnabled(true);
		m_distanceSensor.setAutomaticMode(true);
		m_distanceSensor.setDistanceUnits(Rev2mDistanceSensor.Unit.kInches);
		m_distanceSensor.setRangeProfile(Rev2mDistanceSensor.RangeProfile.kHighAccuracy);

		SmartDashboard.putNumber("Kicker RPM", 0.0);
		SmartDashboard.putNumber("Turret Duty Cycle", 0.0);
		Logger.configureLoggingAndConfig(this, false);
	}

	@Override
	public void robotPeriodic() {
		SmartDashboard.putNumber("Distance Reading (in)", m_distanceSensor.getRange());
		SmartDashboard.putString("Limelight Horizontal", m_limelight.getOffsetHorizontal().toString());
		Logger.updateEntries();
	}

	@Override
	public void disabledInit() {
		m_subsystems.stop();
	}

	@Override
	public void teleopInit() {
		m_subsystems.start();
	}

	@Override
	public void teleopPeriodic() {
		m_turret.set(ControlMode.PercentOutput, m_limelight.getOffsetHorizontal().getRadians() * 0.5);
//		m_turret.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Turret Duty Cycle", 0.0));
		final double speed = SmartDashboard.getNumber("Kicker RPM", 0.0);
		m_shooter.setRPMs(speed, speed);
	}
}
