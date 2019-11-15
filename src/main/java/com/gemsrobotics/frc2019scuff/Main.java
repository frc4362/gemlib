/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.gemsrobotics.frc2019scuff;

import com.gemsrobotics.frc2018.Robot;
import edu.wpi.first.wpilibj.RobotBase;

public final class Main {

    private Main() {
    }

    public static void main(String... args){
        RobotBase.startRobot(ScuffedRobot::new);
    }
}
