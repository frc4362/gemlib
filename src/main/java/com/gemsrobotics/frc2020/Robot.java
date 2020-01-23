package com.gemsrobotics.frc2020;

import com.gemsrobotics.frc2020.subsystems.Chassis;
import com.gemsrobotics.lib.drivers.hid.Gemstick;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.RigidTransformWithCurvature;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.structure.SubsystemManager;
import com.gemsrobotics.lib.subsystems.drivetrain.WheelState;
import com.gemsrobotics.lib.trajectory.TrajectoryContainer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;
import java.util.Collections;

public class Robot extends TimedRobot {
	Gemstick m_stickLeft, m_stickRight;
	Chassis m_chassis;
	SubsystemManager m_subsystemManager;
	TrajectoryContainer<RigidTransformWithCurvature> m_trajectory;

	@Override
	public void robotInit() {
		m_chassis = Chassis.getInstance();

		m_stickLeft = new Gemstick(0);
		m_stickRight = new Gemstick(1);

		m_subsystemManager = new SubsystemManager(m_chassis);

		m_trajectory = m_chassis.getTrajectoryGenerator().generateTrajectory(
				false,
				false,
				Arrays.asList(
						RigidTransform.identity(),
						new RigidTransform(3.0, 0.0, Rotation.identity())),
				Collections.emptyList());
	}

	@Override
	public void robotPeriodic() {
		SmartDashboard.putString("Velocities", m_chassis.getWheelProperty(MotorController::getVelocityLinearMetersPerSecond).toString());
	}

	@Override
	public void disabledInit() {
		m_subsystemManager.stop();
	}

	@Override
	public void teleopInit() {
		m_subsystemManager.start();
		m_chassis.setHighGear(false);
//		Scheduler.getInstance().add(new DriveTrajectoryCommand(m_chassis, m_trajectory));
	}

	@Override
	public void teleopPeriodic() {
//		m_chassis.setCurvatureDrive(
//				deadband(m_stickLeft.y()),
//				deadband(-m_stickRight.x()),
//				m_stickRight.getTrigger());
//		Scheduler.getInstance().run();
		m_chassis.setDriveVelocity(new WheelState(2.0, 2.0));
	}

	private double deadband(final double v) {
		return Math.abs(v) > 0.04 ? v : 0.0;
	}
}
