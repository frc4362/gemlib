package com.gemsrobotics.frc2020;

import com.gemsrobotics.frc2020.subsystems.Chassis;
import com.gemsrobotics.lib.commands.TrackTrajectoryCommand;
import com.gemsrobotics.lib.drivers.hid.Gemstick;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.RigidTransformWithCurvature;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.structure.SubsystemManager;
import com.gemsrobotics.lib.trajectory.TrajectoryContainer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;
import java.util.Collections;

public final class Kitbot extends TimedRobot {
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
						new RigidTransform(2.5, -2.0, Rotation.degrees(-60)),
						new RigidTransform(5.0, -1.7, Rotation.degrees(30)),
						new RigidTransform(8.0, -1.0, Rotation.degrees(60))),
				Collections.emptyList());
	}

	@Override
	public void robotPeriodic() {
		SmartDashboard.putString("Pose", m_chassis.getOdometer().getLatestFieldToVehicleValue().toString());
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
		Scheduler.getInstance().add(new TrackTrajectoryCommand(m_chassis, m_trajectory));
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}
}
