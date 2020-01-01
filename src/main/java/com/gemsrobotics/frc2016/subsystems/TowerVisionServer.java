package com.gemsrobotics.frc2016.subsystems;

import com.gemsrobotics.lib.subsystems.Limelight;

import java.util.Objects;

public final class TowerVisionServer extends Limelight {
	private static TowerVisionServer INSTANCE;

	public static TowerVisionServer getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new TowerVisionServer();
		}

		return INSTANCE;
	}

	private TowerVisionServer() {
		super("tower");
	}

	@Override
	protected synchronized void onCreate(double timestamp) {
		setLEDMode(LEDMode.ON);
	}

	@Override
	protected synchronized  void onEnable(double timestamp) {
		setLEDMode(LEDMode.ON);
	}

	@Override
	protected synchronized void onUpdate(double timestamp) {

	}

	@Override
	protected synchronized void onStop(double timestamp) {
		setLEDMode(LEDMode.OFF);
	}
}
