package com.gemsrobotics.frc2019.subsystems.adjuster;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;

import static edu.wpi.first.wpilibj.GenericHID.Hand.kRight;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

public class LateralAdjusterListener extends Command {
	private final XboxController m_controller;
	private final LateralAdjuster m_adjuster;

	public LateralAdjusterListener(
			final LateralAdjuster adjuster,
			final XboxController controller
	) {
		m_adjuster = adjuster;
		m_controller = controller;
	}

	@Override
	public void execute() {
		final double adjustPower = -m_controller.getX(kRight);

		if (m_controller.getRawButton(9) && abs(m_adjuster.getPosition()) > 200) {
			final double mult = abs(m_adjuster.getPosition()) < 600 ? 0.7 : 1.0,
				power = signum(m_adjuster.getPosition()) * -m_adjuster.kLatVolts * mult;
			m_adjuster.drive(power);
		} else if (abs(adjustPower) > 0.5) {
			m_adjuster.drive(adjustPower * m_adjuster.kLatVolts);
		} else {
			m_adjuster.drive(0);
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
