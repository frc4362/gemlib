package com.gemsrobotics.frc2020;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.frc2020.subsystems.Shooter;
import com.gemsrobotics.lib.structure.SubsystemManager;
import com.gemsrobotics.lib.utils.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;

public class ShooterJig extends TimedRobot implements Loggable {
	@Log
	private Shooter m_shooter;
	private SubsystemManager m_subsystems;

	@Override
	public void robotInit() {
		m_shooter = Shooter.getInstance();
		m_subsystems = new SubsystemManager(m_shooter);

		SmartDashboard.putNumber("Shooter RPM", 0.0);
		Logger.configureLoggingAndConfig(this, false);
	}

	@Override
	public void robotPeriodic() {
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
		m_shooter.setRPMs(SmartDashboard.getNumber("Shooter RPM", 0.0), 0.0);
	}
}
