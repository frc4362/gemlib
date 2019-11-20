package com.gemsrobotics.frc2019.commands;

import com.gemsrobotics.frc2019.Config;
import com.gemsrobotics.frc2019.OperatorInterface;
import com.gemsrobotics.frc2019.subsystems.drive.DifferentialDrive;
import com.gemsrobotics.frc2019.subsystems.drive.DriveCommandConfig;
import com.gemsrobotics.frc2019.util.DualTransmission;
import com.gemsrobotics.frc2019.util.camera.Limelight;
import com.gemsrobotics.frc2019.util.joy.Gemstick;
import com.gemsrobotics.frc2019.util.joy.Gemstick.Lens;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.util.Collection;
import java.util.LinkedList;
import java.util.Queue;

import static java.lang.Math.*;

@SuppressWarnings({"unused", "WeakerAccess"})
public class DriveCommand extends Command {
	private static final double kP = 1.3;
	private static final double SLOWDOWN_PERCENT = 0.5;
	public static final double ADJUSTMENT_THRESHOLD = 0.2;

	private final DifferentialDrive m_chassis;
	private final Gemstick m_stick, m_wheel;
	private final Limelight m_limelight;
	private final SendableChooser<Boolean> m_toggler;
	private final DriveCommandConfig m_cfg;
	private final XboxController m_controller;

	private Queue<DifferentialDrive.DrivePower> m_driveQueue;

	private boolean m_isApproachingLast;

	public DriveCommand(
			final DifferentialDrive chassis,
			final Limelight limelight,
			final OperatorInterface oi,
			final SendableChooser<Boolean> toggler
	) {
		m_toggler = toggler;
		m_chassis = chassis;
		m_stick = oi.getStickLeft();
		m_wheel = oi.getStickRight();
		m_controller = oi.getController();
		m_limelight = limelight;

		m_cfg = Config.getConfig("driveCommand").to(DriveCommandConfig.class);
	}

	private boolean isAttemptingVision() {
		return m_wheel.getRawButton(4);
	}

	private double calculateLimelightAdjustment() {
		return m_limelight.getOffsetHorizontal() * kP;
	}

	private boolean isSlowableArea() {
		return m_limelight.getArea() > SLOWDOWN_PERCENT;
	}

	public void queue(final Collection<DifferentialDrive.DrivePower> velocities) {
		m_driveQueue.addAll(velocities);
	}

	@Override
	public void initialize() {
		m_driveQueue = new LinkedList<>();
	}

	@Override
	public void execute() {
		if (!m_toggler.getSelected()) {
			return;
		}

		if (!m_driveQueue.isEmpty()) {
			final var s = m_driveQueue.poll();
			m_chassis.drive(s);
		} else {
			double linearPower = m_stick.get(Lens.Y);
			final boolean isApproaching = isAttemptingVision() && m_limelight.isTargetPresent();

			final double rotationalPower;
			final boolean isQuickTurn;

			if (isApproaching) {
				isQuickTurn = false;

				// if we're going forwards
				if (linearPower > 0) {
					rotationalPower = calculateLimelightAdjustment();
				} else {
					rotationalPower = m_wheel.get(Lens.X);
				}
			} else {
				isQuickTurn = m_wheel.getRawButton(3);
				rotationalPower =  m_wheel.get(Lens.X);
			}

			// slow drive based on target size
			if (DriverStation.getInstance().isAutonomous() || (isAttemptingVision() && isSlowableArea())) {
				final var previousPower = linearPower;
				final var area = m_limelight.getArea();

				if (area > m_cfg.stopOverdriveThreshold) {
					linearPower = min(0, previousPower);
				} else {
					linearPower = limitLinearPower(m_chassis.getTransmission().get(), area, previousPower);
				}
			}

			boolean isAtTargetHeading = false;
			final var povValue = m_wheel.getPOVState();
			final boolean doSnapTurn = DriverStation.getInstance().isAutonomous() && povValue != Gemstick.POVState.NONE;

			if (doSnapTurn) {
				// this method has side effects
				isAtTargetHeading = m_chassis.turnToHeading(toFieldAngle(povValue));
			}

			if (isAtTargetHeading || !doSnapTurn) {
				m_chassis.curvatureDrive(
						linearPower,
						rotationalPower,
						isQuickTurn
				);
			}
		}
	}

	private double toFieldAngle(final Gemstick.POVState base) {
		switch (base) {
			case N:
				return 0;
			case S:
				return 179;
			case E:
				return 90;
			case W:
				return -90;
			case NE:
//				return 30;
				return 0;
			case NW:
//				return -30;
				return -0;
			case SE:
//				return 150;
				return 0;
			case SW:
//				return -150;
				return -0;
			default:
			case NONE:
				return 0;
		}
	}

	private double limitLinearPower(final DualTransmission.Gear gear, final double area, final double power) {
		final var schema = m_cfg.schemaForGear(gear);
		final var scalingFactor = max(((area - schema.endRampArea) / schema.getDivisor()), 0);
		return min(schema.getRampingRange() * scalingFactor + schema.minimumSpeed, power);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
