package com;

import com.gemsrobotics.lib.math.se2.Rotation;

import static com.gemsrobotics.lib.utils.MathUtils.Tau;
import static com.gemsrobotics.lib.utils.MathUtils.epsilonEquals;
import static java.lang.Math.abs;
import static java.lang.Math.copySign;

public class Test {
	public static void main(String[] args) {
		System.out.println(Rotation.degrees(-179).difference(Rotation.degrees(179)));
	}
}
