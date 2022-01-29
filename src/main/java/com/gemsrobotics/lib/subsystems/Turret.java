package com.gemsrobotics.lib.subsystems;

import com.gemsrobotics.lib.math.se2.Rotation;

public interface Turret {
	void setReference(Rotation reference);
	boolean atReference();
	Rotation getRotation();
}
