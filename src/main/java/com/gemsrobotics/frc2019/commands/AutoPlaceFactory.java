package com.gemsrobotics.frc2019.commands;

import com.gemsrobotics.frc2019.commands.any.Wait;
import com.gemsrobotics.frc2019.subsystems.lift.Lift;
import com.gemsrobotics.frc2019.subsystems.manipulator.Manipulator;
import com.gemsrobotics.frc2019.util.camera.Limelight;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

import java.util.HashMap;
import java.util.Map;

public class AutoPlaceFactory {
	private static final long DEBOUNCE_MS = 1000;
	private static final int ACTUATE_THRESHOLD = 21;

	private final Lift m_lift;
	private final Manipulator m_manipulator;
	private final Map<Lift.Position, Long> m_lastCreationTimes;
	private final Map<Lift.Position, Boolean> m_positionReady;
	private final Limelight m_limelight;

	public AutoPlaceFactory(
			final Lift lift,
			final Manipulator manipulator,
			final Limelight limelight
	) {
		m_lift = lift;
		m_manipulator = manipulator;
		m_limelight = limelight;
		m_lastCreationTimes = new HashMap<>();
		m_positionReady = new HashMap<>();
	}

	public Command makeAutoPlace(
			final Lift.Position position,
			final boolean openAfterFinish,
			final XboxController controller
	) {
		final var currentTime = System.currentTimeMillis();

		final Command ret;

		final var isFirstRun = !m_lastCreationTimes.containsKey(position);
		final var isTimeReady = isFirstRun || (currentTime - m_lastCreationTimes.get(position) > DEBOUNCE_MS);
		final var isPositionReady = m_positionReady.getOrDefault(position, true);

		if (isFirstRun || (isTimeReady && isPositionReady)) {
			m_positionReady.put(position, false);

			ret = new CommandGroup() {
				{
					addSequential(new LiftMovement(m_lift, position));
					addSequential(new Wait(50));
					addSequential(new PlacementSequence(m_manipulator, openAfterFinish));
				}

				@Override
				public void interrupted() {
					end();
				}

				@Override
				public void end() {
					m_positionReady.put(position, true);
				}
			};
//					new WaitForLimelightArea(m_limelight, controller, ACTUATE_THRESHOLD),

			m_lastCreationTimes.put(position, currentTime);
		} else {
			ret = null;
		}

		return ret;
	}
}
