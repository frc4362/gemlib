package com.gemsrobotics.lib.commands;

import com.gemsrobotics.lib.Side;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDrive;
import com.gemsrobotics.lib.subsystems.drivetrain.FieldToVehicleEstimator;
import edu.wpi.first.wpilibj.command.Command;

public class WaitUntilInsideRegionCommand extends Command {
    protected final FieldToVehicleEstimator m_odometer;
    protected final Translation m_cornerLeftBottom, m_cornerRightTop;

    public WaitUntilInsideRegionCommand(
            final DifferentialDrive<?> chassis,
            final Translation bottomLeft,
            final Translation topRight,
            final boolean flip
    ) {
        m_odometer = chassis.getOdometer();

        if (flip) {
            m_cornerLeftBottom = new Translation(bottomLeft.x(), -topRight.y());
            m_cornerRightTop = new Translation(topRight.x(), -bottomLeft.y());
        } else {
            m_cornerLeftBottom = bottomLeft;
            m_cornerRightTop = topRight;
        }
    }

    public WaitUntilInsideRegionCommand(
            final DifferentialDrive<?> chassis,
            final Translation bottomLeft,
            final Translation topRight,
            final Side side
    ) {
        this(chassis, bottomLeft, topRight, side.isRight());
    }

    @Override
    public boolean isFinished() {
        final var currentPosition = m_odometer.getLatestFieldToVehicle().getValue().getTranslation();
        return currentPosition.x() > m_cornerLeftBottom.x()
                && currentPosition.x() < m_cornerRightTop.x()
                && currentPosition.y() > m_cornerLeftBottom.y()
                && currentPosition.y() < m_cornerRightTop.y();
    }
}
