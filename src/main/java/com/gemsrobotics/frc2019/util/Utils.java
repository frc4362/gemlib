package com.gemsrobotics.frc2019.util;

import java.text.DecimalFormat;

public class Utils {
	private Utils() { }

	private static final DecimalFormat m_dcf = new DecimalFormat("#0.000");

	public static DecimalFormat getDCF() {
		return m_dcf;
	}
}
