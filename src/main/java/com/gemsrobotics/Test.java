package com.gemsrobotics;

import com.gemsrobotics.lib.math.se2.Rotation;

public class Test {
	public static void main(String[] args) {
		System.out.println(Rotation.degrees(-179).difference(Rotation.degrees(-180)));
	}
}
