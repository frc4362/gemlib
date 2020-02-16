package com.gemsrobotics.frc2020.subsystems;

import com.gemsrobotics.frc2020.Constants;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.utils.Units;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import io.github.oblarg.oblog.Loggable;

import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;

import static com.gemsrobotics.lib.utils.MathUtils.epsilonEquals;

public final class Hopper extends Subsystem {
	private static final MotorController.GearingParameters GEARING_PARAMETERS =
			new MotorController.GearingParameters(1.0 / 77.575757, Units.inches2Meters(13.75) / 2.0, 1.0);

	private static Hopper INSTANCE;

	public static Hopper getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Hopper();
		}

		return INSTANCE;
	}

	private final MotorController<CANSparkMax> m_motor;
	private final Channel m_channelRight, m_channelCenter, m_channelLeft;
	private final Inventory m_inventory;
	private final ColorSensorV3 m_sensor;
	private final PeriodicIO m_periodicIO;

	private Mode m_mode;

	private Hopper() {
		m_motor = MotorControllerFactory.createSparkMax(Constants.HOPPER_MOTOR_PORT, MotorControllerFactory.DEFAULT_SPARK_CONFIG);
		m_motor.setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
		m_motor.setGearingParameters(GEARING_PARAMETERS);
		m_motor.setSelectedProfile(0);
		m_motor.setPIDF(12.1, 0.0, 0.979, 0.0);
//		m_motor.setClosedLoopVoltageRampRate(0.1);
		m_motor.setEncoderRotations(0.0);

		m_channelRight = new Channel(Inventory.Location.RIGHT_INTAKE, Constants.CHANNEL_RIGHT_PORT);
		m_channelCenter = new Channel(Inventory.Location.CENTER_INTAKE, Constants.CHANNEL_CENTER_PORT);
		m_channelLeft = new Channel(Inventory.Location.LEFT_INTAKE, Constants.CHANNEL_LEFT_PORT);

		m_inventory = new Inventory();
		m_sensor = new ColorSensorV3(I2C.Port.kMXP);
		m_periodicIO = new PeriodicIO();

		m_mode = Mode.DISABLED;
	}

	public enum Mode {
		DISABLED,
		INTAKING,
		CLEARING,
		SURVEYING
	}

	private static class PeriodicIO implements Loggable {
		public double referenceRotations = 0.0;
		public double positionRotations = 0.0;
		public Rotation velocityPerSecond = Rotation.identity();
		public boolean atReference = false;

		public ColorSensorV3.RawColor observedColor = new ColorSensorV3.RawColor(0, 0, 0, 0);
	}

	@Override
	protected synchronized void readPeriodicInputs(final double timestamp) {
		doForChannels(channel -> channel.readPeriodicInputs(timestamp));

//		m_periodicIO.observedColor = m_sensor.getRawColor();
		m_periodicIO.velocityPerSecond = Rotation.radians(m_motor.getVelocityAngularRadiansPerSecond());
		m_periodicIO.positionRotations = m_motor.getPositionRotations();
		m_periodicIO.atReference =
			epsilonEquals(m_periodicIO.referenceRotations, m_periodicIO.positionRotations, (1.0 / 360.0))
			&& epsilonEquals(m_periodicIO.velocityPerSecond.getDegrees(), 0.0, 0.5);
	}

	private void doForChannels(final Consumer<Channel> op) {
		for (final var channel : List.of(m_channelRight, m_channelCenter, m_channelLeft)) {
			op.accept(channel);
		}
	}

	private synchronized void rotate(final int steps) {
		m_periodicIO.referenceRotations = m_periodicIO.referenceRotations + (1.0 / 6.0) * steps;
	}

	@Override
	protected synchronized void onCreate(final double timestamp) {
		doForChannels(channel -> channel.onCreate(timestamp));
	}

	@Override
	protected synchronized void onUpdate(final double timestamp) {
		doForChannels(channel -> {
			channel.onUpdate(timestamp);

			if (channel.isBallAtBottom() && !channel.isBusy()) {
				channel.carryBallToTop();
			}
		});

		switch (m_mode) {
			case RATCHETING:
				m_motor.setPositionRotations(m_periodicIO.referenceRotations);

				if (m_periodicIO.atReference) {
					m_inventory.setRotations(m_periodicIO.positionRotations);

					if (m_inventory.getFilledChamberCount() == 6) {
						return;
					}

					Channel loadChannel = null;

					if (m_channelRight.isBallAtTop()) {
						loadChannel = m_channelRight;
					} else if (m_channelCenter.isBallAtTop()) {
						loadChannel = m_channelCenter;
					} else if (m_channelLeft.isBallAtTop()) {
						loadChannel = m_channelLeft;
					}

					if (!Objects.isNull(loadChannel)) {
						final var optimalChamber = m_inventory.getOptimalLoadingChamber(loadChannel.getLocation()).get();
						final var movement = optimalChamber.getDistance(m_inventory.getNearestChamber(loadChannel.getLocation()));

						if (movement == 0) {
							loadChannel.putBallInHopper();
						} else {
							rotate(movement);
						}
					}
				}

				break;
			default:
				m_motor.setDutyCycle(0.0);
				break;
		}
	}

	@Override
	protected synchronized void onStop(final double timestamp) {
		doForChannels(channel -> channel.onStop(timestamp));
	}

	@Override
	public void setSafeState() {

	}

	public synchronized int
}
