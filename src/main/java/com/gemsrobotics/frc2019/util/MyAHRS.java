package com.gemsrobotics.frc2019.util;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * Wrapper class for our AHRS/Nav MXP
 */
@SuppressWarnings({"unused", "WeakerAccess"})
public class MyAHRS extends AHRS implements Sendable {
	public MyAHRS(final SPI.Port id) {
		super(id);
	}

	public static double boundHalfDegrees(double degrees) {
		while (degrees >= 180.0) {
			degrees -= 360.0;
		}

		while (degrees < -180.0) {
			degrees += 360.0;
		}

		return degrees;
	}

	/**
	 * @return 0-359
	 */
	public static double reducedHeading(double ret) {
		while (ret < 0) {
			ret += 360;
		}

		while (ret >= 360) {
			ret -= 360;
		}

		return ret;
	}

	/**
	 * @return [-180,+180] degrees instead of infinite in either direction
	 */
	public double getHalfAngle() {
		return boundHalfDegrees(getAngle());
	}

	/**
	 * Creates a real-time logging object for use on OutlineViewer/LiveWindow
	 */
	@Override
	public void initSendable(final SendableBuilder builder) {
		builder.setSmartDashboardType("AHRS-Heading");
		builder.setUpdateTable(() ->
		   builder.getEntry("Heading").setDouble(getHalfAngle()));
	}
}
