package com.gemsrobotics.frc2019.subsystems.inventory;

import edu.wpi.first.wpilibj.Preferences;

public class PreferencesInventory extends Inventory {
	private final Preferences m_prefs;

	public PreferencesInventory() {
		m_prefs = Preferences.getInstance();
	}

	@Override
	public boolean hasCargo() {
		return m_prefs.getBoolean("cargo?", false);
	}

	@Override
	public boolean hasPanel() {
		return m_prefs.getBoolean("panel?", false);
	}
}
