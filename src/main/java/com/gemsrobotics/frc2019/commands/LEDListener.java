package com.gemsrobotics.frc2019.commands;

import com.gemsrobotics.frc2019.Hardware;
import com.gemsrobotics.frc2019.subsystems.inventory.Inventory;
import com.gemsrobotics.frc2019.subsystems.inventory.ManualInventory;
import com.gemsrobotics.frc2019.util.camera.Limelight;
import com.gemsrobotics.frc2019.util.joy.Gemstick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDListener extends Command {
    private final Relay m_light;
    private final XboxController m_controller;
    private final Limelight m_limelight;
    private final Inventory m_inventory;

    public LEDListener(
            final Relay light,
            final XboxController controller,
            final Limelight limelight,
            final Inventory inventory
    ) {
        m_light = light;
        m_controller = controller;
        m_limelight = limelight;
        m_inventory = inventory;
    }

    @Override
    public void execute() {
        if (m_inventory instanceof ManualInventory) {
            m_light.set(m_controller.getRawButton(3) ? Relay.Value.kOn : Relay.Value.kOff);
        } else {
            final var isAttemptingPickup = m_controller.getPOV() == Gemstick.POVState.W.getValue();

            final boolean isHandClosed = Hardware.getInstance().getManipulator().getHand().get();

            SmartDashboard.putBoolean("hand closed", isHandClosed);

            if (m_inventory.getCurrentPiece() == Inventory.GamePiece.CARGO || isHandClosed) {
                m_light.set(Relay.Value.kOn);
            } else {
                m_light.set(Relay.Value.kOff);
            }

//            if (m_controller.getPOV() == -1) {
//                m_light.set(Relay.Value.kOff);
//            } else if (isAttemptingPickup && m_limelight.isTargetPresent()) {
//                m_light.set(Relay.Value.kOn);
//            } else if (isAttemptingPickup) {
//                m_light.set((System.currentTimeMillis() / 300L) % 2 == 0 ? Relay.Value.kOn : Relay.Value.kOff);
//            }
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
