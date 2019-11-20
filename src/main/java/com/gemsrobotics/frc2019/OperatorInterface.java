package com.gemsrobotics.frc2019;

import com.gemsrobotics.frc2019.commands.AutoPickupCommand;
import com.gemsrobotics.frc2019.commands.AutoPlaceFactory;
import com.gemsrobotics.frc2019.commands.ClimberRollerListener;
import com.gemsrobotics.frc2019.subsystems.lift.Lift;
import com.gemsrobotics.frc2019.subsystems.manipulator.Manipulator;
import com.gemsrobotics.frc2019.subsystems.manipulator.Manipulator.RunMode;
import com.gemsrobotics.frc2019.util.command.Commands;
import com.gemsrobotics.frc2019.util.joy.Gembutton;
import com.gemsrobotics.frc2019.util.joy.Gemstick;
import com.gemsrobotics.frc2019.util.joy.Gemstick.POVState;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.POVButton;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;

import java.util.Objects;

import static com.gemsrobotics.frc2019.util.command.Commands.commandOf;

@SuppressWarnings({"unused", "WeakerAccess"})
public final class OperatorInterface {
	private final Gemstick m_stickLeft, m_stickRight;
	private final XboxController m_controller;
	private final ClimberRollerListener m_rollerListener;
	private final Lift m_lift;
	private final Manipulator m_manipulator;

	private AutoPlaceFactory m_factory;

	public AutoPlaceFactory getAutoFactory() {
		return m_factory;
	}

	public OperatorInterface(
			final int portLeft,
			final int portRight,
			final int portController
	) {
		m_stickLeft = new Gemstick(portLeft);
		m_stickRight = new Gemstick(portRight);
		m_controller = new XboxController(portController);

		final var hw = Hardware.getInstance();

		final Gembutton
				shiftDownButton = new Gembutton(m_stickLeft, 1),
				shiftUpButton = new Gembutton(m_stickRight, 1),
				intakeButton = new Gembutton(m_controller, 1),
				exhaustButton = new Gembutton(m_controller, 4),
				handButton = new Gembutton(m_controller, 5),
				armButton = new Gembutton(m_controller, 6),
				cargoHeightButton = new Gembutton(m_controller, 7),
				frontLegsButton = new Gembutton(m_stickRight, 9),
				backLegsButton = new Gembutton(m_stickRight, 11),
				stage1ExtendButton = new Gembutton(m_controller, 2);

		final Gembutton
				limelightButton = new Gembutton(m_stickRight, 4);

		final POVButton
				autoPickupButton = new POVButton(m_controller, POVState.W.toDegrees()),
				height1Button = new POVButton(m_controller, POVState.S.toDegrees()),
				height2Button = new POVButton(m_controller, POVState.E.toDegrees()),
				height3Button = new POVButton(m_controller, POVState.N.toDegrees());

		m_rollerListener = new ClimberRollerListener(hw.getRollers(), m_controller);

		stage1ExtendButton.whenPressed(() ->
			hw.getStage1Solenoid().set(true));
		stage1ExtendButton.whenReleased(() ->
			hw.getStage1Solenoid().set(false));

		final Trigger ptoDeployButton = new Trigger() {
			@Override
			public boolean get() {
				final var ds = DriverStation.getInstance();
				return (!ds.isAutonomous() || ds.getMatchTime() < 30) && m_stickRight.getRawButton(7);
			}
		};

		ptoDeployButton.whenActive(commandOf(() -> {
			hw.getPTO().engage();
			hw.getBackLegs().set(DoubleSolenoid.Value.kReverse);
			hw.getManipulator().setSetSpeed(RunMode.HALTED);
			hw.getLateralAdjuster().disable();

			if (!m_rollerListener.hasPreviouslyRun()) {
				Scheduler.getInstance().add(m_rollerListener);
			}
		}));

		frontLegsButton.whenPressed(commandOf(() -> {
		  	  if (hw.getPTO().isEngaged()) {
				  hw.getPTO().disengage();
			  }
		}));

		backLegsButton.whenPressed(() ->
		      hw.getBackLegs().set(DoubleSolenoid.Value.kReverse));

		backLegsButton.whenReleased(() ->
			  hw.getBackLegs().set(DoubleSolenoid.Value.kForward));

		final Runnable intakeNeutralizer = () ->
			  hw.getManipulator().setSetSpeed(RunMode.NEUTRAL);

		final var limelight = hw.getLimelight();
		final var transmission = hw.getChassis().getTransmission();
		final var inventory = hw.getInventory();

		m_manipulator = hw.getManipulator();
		m_lift = hw.getLift();

		final var stage1 = hw.getStage1Solenoid();

		intakeButton.whenPressed(() -> {
			if (m_lift.getPosition() > m_lift.heightRotations(Lift.Position.STAGE1_RETRACT_DISTANCE)) {
				m_manipulator.setSetSpeed(RunMode.INTAKING_RAISED);
			} else {
				stage1.set(true);
				m_manipulator.setSetSpeed(RunMode.INTAKING);
			}
		});
		intakeButton.whenReleased(() -> {
			intakeNeutralizer.run();
			stage1.set(false);
		});

		exhaustButton.whenPressed(() -> {
			if (!(m_lift.getPosition() > m_lift.heightRotations(Lift.Position.STAGE1_RETRACT_DISTANCE))) {
				m_manipulator.setSetSpeed(RunMode.EXHAUSTING);
			} else {
				m_manipulator.setSetSpeed(RunMode.EXHAUSTING_RAISED);
			}
		});
		exhaustButton.whenReleased(intakeNeutralizer);

		handButton.whenPressed(() ->
			m_manipulator.getHand().set(true));
		handButton.whenReleased(() ->
			m_manipulator.getHand().set(false));
		armButton.whenPressed(() ->
			m_manipulator.getArm().set(true));
		armButton.whenReleased(() ->
		    m_manipulator.getArm().set(false));

		final var autoPickupSequence = new AutoPickupCommand(m_manipulator, limelight, hw.getChassis());

		autoPickupButton.whenPressed(autoPickupSequence);
		autoPickupButton.whenReleased(commandOf(() -> {
			if (autoPickupSequence.isRunning()) {
				m_manipulator.getArm().set(false);
				m_manipulator.getHand().set(false);
			}

			autoPickupSequence.cancel();
		}));

		final var autoPlaceFactory = new AutoPlaceFactory(m_lift, m_manipulator, limelight);

		cargoHeightButton.whenPressed(commandOf(() -> {
			if (m_controller.getRawButton(1) || m_controller.getRawButton(2)) {
				m_lift.setPosition(Lift.Position.CARGO_1);
			} else {
				m_lift.setPosition(Lift.Position.CARGO_SHIP);
			}
		}));

		generatePlacementBindings(
				autoPlaceFactory,
				height1Button,
				Lift.Position.PANEL_1,
				Lift.Position.CARGO_1);

		generatePlacementBindings(
				autoPlaceFactory,
				height2Button,
				Lift.Position.PANEL_2,
				Lift.Position.CARGO_2);

		generatePlacementBindings(
				autoPlaceFactory,
				height3Button,
				Lift.Position.PANEL_3,
				Lift.Position.CARGO_3);
	}

	public void generatePlacementBindings(
			final AutoPlaceFactory autoPlaceFactory,
			final Button button,
			final Lift.Position panelPosition,
			final Lift.Position cargoPosition
	) {
		final Command[] panelPlacementCommand = { null };
		final boolean[] isPressedInAuton = { false };

		button.whenPressed(commandOf(() -> {
			final var piece = Hardware.getInstance().getInventory().getCurrentPiece();
			isPressedInAuton[0] = DriverStation.getInstance().isAutonomous();

			switch (piece) {
				case PANEL:
					// manual lift height setpoint override
					if (m_controller.getRawButton(8)) {
						m_lift.setPosition(panelPosition);
						panelPlacementCommand[0] = null;
					} else {
						panelPlacementCommand[0] = autoPlaceFactory.makeAutoPlace(
								panelPosition,
								false,
								m_controller);

						if (!Objects.isNull(panelPlacementCommand[0])) {
							Scheduler.getInstance().add(panelPlacementCommand[0]);
						}
					}
					break;
				case CARGO:
					panelPlacementCommand[0] = null;
					m_lift.setPosition(cargoPosition);
					break;
				default:
					break;
			}
		}));

		button.whenReleased(commandOf(() -> {
			final Runnable reset = () -> {
				m_manipulator.getHand().set(false);

				if (!m_controller.getRawButton(8) && !isPressedInAuton[0] && panelPosition != Lift.Position.PANEL_1) {
					m_lift.setPosition(Lift.Position.BOTTOM);
				}
			};

			if (!Objects.isNull(panelPlacementCommand[0])) {
				// queues opening the hand
				if (panelPlacementCommand[0].isRunning()) {
					Scheduler.getInstance().add(
							Commands.listenForFinish(panelPlacementCommand[0], commandOf(reset)));
				} else {
					reset.run();
				}
			}
		}));
	}

	public void resetControls() {
		m_rollerListener.reset();
	}

	public Gemstick getStickLeft() {
		return m_stickLeft;
	}

	public Gemstick getStickRight() {
		return m_stickRight;
	}

	public XboxController getController() {
		return m_controller;
	}
}
