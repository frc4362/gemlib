package com.gemsrobotics.frc2019.commands;

import com.gemsrobotics.frc2019.subsystems.adjuster.LateralAdjuster;
import com.gemsrobotics.frc2019.subsystems.inventory.Inventory;
import com.gemsrobotics.frc2019.subsystems.lift.Lift;
import com.gemsrobotics.frc2019.util.camera.Limelight;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;

import java.util.List;
import java.util.Optional;

import static java.lang.Math.abs;
public class VisionAdjuster extends Command {
	private static final double OVERRIDE_THRESHOLD = 0.3;

	private final LateralAdjuster m_adjuster;
	private final Limelight m_limelight;
	private final Inventory m_inventory;
	private final XboxController m_controller;
	private final Lift m_lift;

	private boolean m_isOverridingLast;

	public VisionAdjuster(
			final LateralAdjuster lateralAdjuster,
			final Limelight limelight,
			final Inventory inventory,
			final XboxController controller,
			final Lift lift
	) {
		m_adjuster = lateralAdjuster;
		m_limelight = limelight;
		m_inventory = inventory;
		m_controller = controller;
		m_lift = lift;
	}

	@Override
	public void initialize() {
		m_isOverridingLast = false;
	}

	@Override
	public void execute() {
		final double driveVal = deadband(m_controller.getX(GenericHID.Hand.kRight));

		boolean isOverriding = m_isOverridingLast || driveVal != 0.0;

		if (isOverriding) {
			m_adjuster.drive(driveVal * -m_adjuster.kLatVolts);
		} else if (m_controller.getRawButton(1)) { // when you're intaking cargo
			m_adjuster.setPercent(0.5);
		} else if (m_lift.isBlockingCamera()) {
			m_adjuster.drive(0);
		} else if (m_limelight.isTargetPresent() && !m_inventory.hasCargo()) {
			double extraAdjustment = 0;

//			final var yaw = m_ahrs.getHalfAngle();
//			final var target = Optional.ofNullable(Target.forAngle(yaw));
//
//			if (target.map(Target::useYawAdjustment).orElse(false)) {
//			if (m_controller.getPOV() != Gemstick.POVState.W.getValue()) {
//				final var error = calculateAngle(m_limelight);
//				final var deltaX = (9.5 - (9.5 * cos(error))) / cos(error);
//				extraAdjustment = m_adjuster.inches2Ticks(deltaX * cos(error));
//			}
//			}

			final var proportion = (-m_limelight.getOffsetHorizontal() / m_adjuster.getAdjustmentThreshold() + 1) / 2;
			m_adjuster.setTicks((proportion * m_adjuster.getWidthTicks() - (m_adjuster.getWidthTicks() / 2)) + extraAdjustment);
//			m_adjuster.alignRadians(-m_limelight.getOffsetHorizontal());
		}

		if (m_isOverridingLast && m_controller.getRawButton(9)) {
			isOverriding = false;
		}

		m_isOverridingLast = isOverriding;
	}

	private double calculateAngle(final Limelight limelight) {
		final Optional<Double> tx = limelight.getRawProperty("tx"),
				ta0 = limelight.getRawProperty("ta0"),
				ta1 = limelight.getRawProperty("ta1"),
				tx0 = limelight.getRawProperty("tx0"),
				tx1 = limelight.getRawProperty("tx1");

		if (List.of(tx, ta0, ta1, tx0, tx1).stream().anyMatch(Optional::isEmpty)) {
			return 0;
		}

		// as a consequence, if you are not pointed at the target, it simply will not work
		// even if you need it to
		if (abs(tx.get()) > 10) {
			return 0;
		}

		final var txNormalized = tx.get() / 29.8;

		final double txL, txR, taL, taR;

		if (tx0.get() > txNormalized && tx1.get() < txNormalized) {
			txR = tx0.get();
			txL = tx1.get();
			taR = ta0.get();
			taL = ta1.get();
		} else {
			txR = tx1.get();
			txL = tx0.get();
			taR = ta1.get();
			taL = ta0.get();
		}

		final var left2right = taL / taR;

		final double
				a = -9.759266958,
				b = 55.87285847,
				c = -50.40638987;

		return a * left2right * left2right + b * left2right + c;
	}

	private static double deadband(final double val) {
		return abs(val) > OVERRIDE_THRESHOLD ? val : 0.0;
	}

	@Override
	public boolean isFinished() {
		return m_adjuster.isDisabled();
	}
}
