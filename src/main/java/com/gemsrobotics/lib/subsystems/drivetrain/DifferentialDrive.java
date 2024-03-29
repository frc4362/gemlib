package com.gemsrobotics.lib.subsystems.drivetrain;

import com.gemsrobotics.lib.controls.DriveMotionPlanner;
import com.gemsrobotics.lib.controls.MotionPlanner;
import com.gemsrobotics.lib.controls.PIDFController;
import com.gemsrobotics.lib.drivers.imu.NavX;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerGroup;
import com.gemsrobotics.lib.drivers.transmission.DualSpeedTransmission;
import com.gemsrobotics.lib.drivers.transmission.Transmission;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.RigidTransformWithCurvature;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.physics.MotorModel;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.trajectory.TrajectoryIterator;
import com.gemsrobotics.lib.trajectory.parameterization.TimedState;
import com.gemsrobotics.lib.controls.DriveMotionPlanner.FollowerType;
import com.gemsrobotics.lib.trajectory.parameterization.TrajectoryGenerator;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.Objects;
import java.util.Optional;
import java.util.function.Function;

// just copy what was done here
// https://github.com/frc1678/robot-code-public/blob/master/c2019/subsystems/drivetrain/drivetrain_base.cpp
// when extending it for a given implementation
@SuppressWarnings({"unused", "WeakerAccess", "OverlyCoupledClass"})
public abstract class DifferentialDrive<MotorType> extends Subsystem {
    // This config as one class allows for deserialization from a JSON file to describe nearly the entire drive train
    public static class Config {
		public double maxVoltage;
		public double secondsToMaxVoltage;

		public MotorController.GearingParameters gearingLowGear;
		public MotorController.GearingParameters gearingHighGear;

		public PIDFController.Gains gainsLowGear;
		public PIDFController.Gains gainsHighGear;

		public MotorModel.Properties propertiesLowGear;
		public MotorModel.Properties propertiesHighGear;
		public DifferentialDriveModel.Properties propertiesModel;
		public MotionPlanner.MotionConfig motionConfig;

		public OpenLoopDriveHelper.Config openLoopConfig;

		private PIDFController.Gains velocityGainsForGear(final boolean highGear) {
		    return highGear ? gainsHighGear : gainsLowGear;
        }

        private MotorController.GearingParameters gearingForGear(final boolean highGear) {
			return highGear ? gearingHighGear : gearingLowGear;
		}
	}

	protected final Config m_config;
    protected final DifferentialDriveModel m_model;
	protected final OpenLoopDriveHelper m_openLoopHelper;
    protected final TrajectoryGenerator m_generator;
	protected final DriveMotionPlanner m_motionPlanner;

    protected final NavX m_imu;
	protected final MotorControllerGroup<MotorType> m_motorsLeft, m_motorsRight;
    protected final MotorController<MotorType> m_masterMotorLeft, m_masterMotorRight;
	protected final Transmission m_transmission;
    protected final PeriodicIO m_periodicIO;

    protected FieldToVehicleEstimator m_odometer;
    protected boolean m_isHighGear, m_forceFinishTrajectory;
	protected ControlMode m_controlMode;
	protected Rotation m_headingOffset;

    protected abstract Config getConfig();
    // Invert motors BEFORE passing them into the method...
    protected abstract MotorControllerGroup<MotorType> getMotorControllersLeft();
    protected abstract MotorControllerGroup<MotorType> getMotorControllersRight();
    protected abstract Transmission getTransmission();

	protected DifferentialDrive() {
        m_config = getConfig();
		m_transmission = getTransmission();
        m_motorsLeft = getMotorControllersLeft();
        m_motorsRight = getMotorControllersRight();

        m_masterMotorLeft = m_motorsLeft.getMaster();
        m_masterMotorRight = m_motorsRight.getMaster();

        configureMotorController(m_masterMotorLeft);
        m_motorsLeft.followMaster(false);

        configureMotorController(m_masterMotorRight);
        m_motorsRight.followMaster(false);

        m_imu = new NavX();
		m_model = new DifferentialDriveModel(
		        m_config.propertiesModel,
                new MotorModel(m_config.propertiesLowGear),
                new MotorModel(m_config.propertiesHighGear));
        m_openLoopHelper = new OpenLoopDriveHelper(m_config.openLoopConfig);
        m_generator = new TrajectoryGenerator(m_config.motionConfig, m_model);
		m_motionPlanner = new DriveMotionPlanner(m_config.motionConfig, m_model, FollowerType.FEEDFORWARD);
        m_periodicIO = new PeriodicIO();

		m_odometer = FieldToVehicleEstimator.withStarting(m_model, 0.0, RigidTransform.identity());
        m_headingOffset = Rotation.identity();
		m_forceFinishTrajectory = false;
    }

	private void configureMotorController(final MotorController<MotorType> controller) {
		controller.setOpenLoopVoltageRampRate(m_config.secondsToMaxVoltage);
		controller.setGearingParameters(m_config.gearingForGear(m_transmission instanceof DualSpeedTransmission
			&& ((DualSpeedTransmission) m_transmission).isInverted()));

		controller.setSelectedProfile(slotForGear(false));
		controller.setPIDF(m_config.gainsLowGear);

		controller.setEncoderCounts(0.0);
	}

	public enum ControlMode {
		DISABLED,
		OPEN_LOOP,
		VELOCITY,
        TRAJECTORY_TRACKING,
		TUNING,
		VOLTAGE
	}

	private final class PeriodicIO implements Loggable {
		// Inputs
        @Log.ToString(name="Wheel Position (m, m)")
		public WheelState positionMeters = new WheelState();
        @Log.ToString(name="Wheel Delta Position (m, m)")
		public WheelState positionDeltaMeters = new WheelState();
        @Log.ToString(name="Wheel Velocity (m per s, m per s)")
		public WheelState velocityMeters = new WheelState();
        @Log.ToString(name="Wheel Acceleration (m per s^2, m per s^2)")
		public WheelState accelerationMeters = new WheelState();
        @Log.ToString(name="Heading (deg)")
		public Rotation heading = Rotation.identity();
        @Log(name="High Gear? (Boolean)")
		public boolean isHighGear = m_transmission.isHighGear();

        @Log(name="Tipping? (Boolean)")
		public boolean isTipping = false;
        @Log(name="Colliding? (Boolean)")
		public boolean isCollisionOccurring = false;

		// Outputs
        @Log.ToString(name="Control Mode")
        public ControlMode demandType = null;
        @Log.ToString(name="Current Demand")
		public WheelState demand = new WheelState();
        @Log.ToString(name="Feedforward Demand")
		public WheelState feedforward = new WheelState();

		public RigidTransform trackingError = null;
		public TimedState<RigidTransformWithCurvature> trackingReference = null;
	}

    public synchronized boolean setNeutralBehaviour(final MotorController.NeutralBehaviour mode) {
        // please note the use of the NON short-circuiting operator (&&)
		final var left = m_motorsLeft.forEachAttempt(motor -> motor.setNeutralBehaviour(mode));
		final var right = m_motorsRight.forEachAttempt(motor -> motor.setNeutralBehaviour(mode));
        return left && right;
    }

	public synchronized void setHighGear(final boolean wantsHighGear) {
		if (wantsHighGear != m_isHighGear) {
			m_isHighGear = wantsHighGear;
			m_transmission.setHighGear(wantsHighGear);

            final int profile = slotForGear(wantsHighGear);
            m_masterMotorLeft.setSelectedProfile(profile);
            m_masterMotorRight.setSelectedProfile(profile);

            final MotorController.GearingParameters gearingParameters = m_config.gearingForGear(wantsHighGear);
            m_masterMotorLeft.setGearingParameters(gearingParameters);
            m_masterMotorRight.setGearingParameters(gearingParameters);
		}
	}

	public synchronized void setDisabled() {
		configureControlMode(ControlMode.DISABLED);
	}

	protected synchronized void configureControlMode(final ControlMode newControlMode) {
	    if (newControlMode != m_controlMode) {
	    	m_periodicIO.demand = new WheelState();
			m_periodicIO.feedforward = new WheelState();
			m_periodicIO.trackingError = null;
			m_periodicIO.trackingReference = null;

	        switch (newControlMode) {
                case DISABLED:
					setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);

                    m_controlMode = ControlMode.DISABLED;
                    break;
                case OPEN_LOOP:
				case VOLTAGE:
                    setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);

                    m_controlMode = newControlMode;
                    break;
				case VELOCITY:
                case TRAJECTORY_TRACKING:
                    setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);

                    m_forceFinishTrajectory = false;
                    m_motionPlanner.reset();

                    m_controlMode = ControlMode.TRAJECTORY_TRACKING;
                    break;
				case TUNING:
					setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
					m_controlMode = ControlMode.TUNING;
					break;
            }
        }
    }

	public synchronized void setCurvatureDrive(final double throttle, final double wheel, final boolean isQuickTurn) {
		configureControlMode(ControlMode.OPEN_LOOP);
		m_periodicIO.demand = m_openLoopHelper.drive(throttle, wheel, isQuickTurn, m_periodicIO.isHighGear);
	}

	public synchronized void setOpenLoop(final ChassisState chassisState) {
	    setOpenLoop(new WheelState(chassisState.linear - chassisState.angular, chassisState.linear + chassisState.angular));
    }

    public synchronized void setOpenLoop(final WheelState wheelState) {
		configureControlMode(ControlMode.OPEN_LOOP);
		m_periodicIO.demand = wheelState;
	}

	public void setVoltages(final WheelState voltages) {
		configureControlMode(ControlMode.VOLTAGE);
		m_periodicIO.demand = voltages;
	}

	public synchronized void setTrajectory(final TrajectoryIterator<TimedState<RigidTransformWithCurvature>> trajectory) {
		configureControlMode(ControlMode.TRAJECTORY_TRACKING);
		m_motionPlanner.setTrajectory(trajectory);
		m_motionPlanner.reset();
	}

	public synchronized void setDriveVelocity(final WheelState velocities) {
		configureControlMode(ControlMode.VELOCITY);
		m_periodicIO.demand = velocities;
	}

	public synchronized void setHoldCurrentSpeeds() {
		setDriveVelocity(m_periodicIO.velocityMeters);
	}

	public synchronized void setTuningValues(final WheelState demand, final double kP, final double kD) {
		configureControlMode(ControlMode.TUNING);
		m_motorsLeft.getMaster().setPIDF(kP, 0.0, kD, 0.0);
		m_periodicIO.demand = demand;
	}

	protected final void driveOpenLoop(final WheelState demandDutyCycle) {
		m_masterMotorLeft.setDutyCycle(demandDutyCycle.left);
		m_masterMotorRight.setDutyCycle(demandDutyCycle.right);
	}

	protected final void driveVelocity(final WheelState demandMetersPerSecond, final WheelState feedforwardDutyCycle) {
		m_masterMotorLeft.setVelocityMetersPerSecond(demandMetersPerSecond.left, feedforwardDutyCycle.left);
		m_masterMotorRight.setVelocityMetersPerSecond(demandMetersPerSecond.right, feedforwardDutyCycle.right);
	}

	@Override
	protected synchronized void readPeriodicInputs(final double timestamp) {
        m_periodicIO.demandType = m_controlMode;

        // Position calculations
		final var oldPosition = new WheelState(m_periodicIO.positionMeters);
		final var newPosition = getWheelProperty(MotorController::getPositionMeters);
		m_periodicIO.positionMeters = newPosition;
		m_periodicIO.positionDeltaMeters = newPosition.difference(oldPosition);

		// Velocity and acceleration calculations
		final var oldVelocity = new WheelState(m_periodicIO.velocityMeters);
		final var newVelocity = getWheelProperty(MotorController::getVelocityLinearMetersPerSecond);
		m_periodicIO.velocityMeters = newVelocity;
		m_periodicIO.accelerationMeters = newVelocity.difference(oldVelocity).map(dv -> dv / dt());

		// IMU reads and adjustments
		m_periodicIO.heading = m_imu.getFusedHeading().rotateBy(m_headingOffset);
        m_periodicIO.isCollisionOccurring = m_imu.isCollisionOccurring();
        m_periodicIO.isTipping = m_imu.isTipping();

        // Transmission
        m_periodicIO.isHighGear = m_transmission.isHighGear();
	}

	protected void updateStateEstimation(final double timestamp) {
		final var predictedVelocity = m_model.forwardKinematics(m_periodicIO.velocityMeters.left, m_periodicIO.velocityMeters.right);
		m_odometer.addObservation(timestamp, m_periodicIO.positionDeltaMeters, m_periodicIO.heading, predictedVelocity);
	}

	protected void updateTrajectoryFollowingDemands(final double timestamp) {
		final var output = m_motionPlanner.update(timestamp, m_odometer.getFieldToVehicle(timestamp), m_periodicIO.isHighGear)
								   .orElse(new MotionPlanner.Output());

		m_periodicIO.trackingError = m_motionPlanner.getError();
		m_periodicIO.trackingReference = m_motionPlanner.getReference();

		if (m_forceFinishTrajectory) {
			setDisabled();
		} else {
			// convert from radians/second to meters/second
			m_periodicIO.demand = output.velocityRadiansPerSecond.map(radiansPerSecond -> (m_config.propertiesModel.wheelRadiusMeters * radiansPerSecond));
			m_periodicIO.feedforward = output.feedforwardVoltage.map(v -> v / 12.0);
		}
	}

	@Override
	protected final synchronized void onStart(final double timestamp) {
		setDisabled();
	}

	@Override
	protected synchronized void onUpdate(final double timestamp) {
        updateStateEstimation(timestamp);

        switch (m_controlMode) {
            case DISABLED:
                // No actuation
				m_masterMotorLeft.setNeutral();
				m_masterMotorRight.setNeutral();
                break;
            case OPEN_LOOP:
                // This makes sense, since this demand was updated externally
                driveOpenLoop(m_periodicIO.demand);
                break;
			case VOLTAGE:
				m_masterMotorLeft.setVoltage(m_periodicIO.demand.left);
				m_masterMotorRight.setVoltage(m_periodicIO.demand.right);
				break;
            case TRAJECTORY_TRACKING:
                updateTrajectoryFollowingDemands(timestamp);
			case VELOCITY: // note fallthrough
                driveVelocity(m_periodicIO.demand, m_periodicIO.feedforward);
                break;
			case TUNING:
				final var FF = m_model.solveInverseDynamics(
						m_periodicIO.demand.map(s -> s / m_model.wheelRadiusMeters),
						m_periodicIO.demand.difference(m_periodicIO.velocityMeters.map(s -> s / m_model.wheelRadiusMeters)),
						m_periodicIO.isHighGear).voltage;
				driveVelocity(m_periodicIO.demand, FF);
				break;
        }
	}

	@Override
	public void setSafeState() {
		setDisabled();
		setNeutralBehaviour(MotorController.NeutralBehaviour.COAST);
	}

	@Override
	public FaultedResponse checkFaulted() {
	    var response = FaultedResponse.NONE;

	    if (!m_masterMotorLeft.isEncoderPresent()) {
	        System.out.println("HARDWARE FAULT: Left encoder lost!!");
	        response = FaultedResponse.DISABLE_SUBSYSTEM;
        }

	    if (!m_masterMotorRight.isEncoderPresent()) {
	        System.out.println("HARDWARE FAULT: Right encoder lost!!");
            response = FaultedResponse.DISABLE_SUBSYSTEM;
        }

	    return response;
	}

    private static int slotForGear(final boolean isHighGear) {
	    return isHighGear ? 1 : 0;
    }

	public synchronized final boolean isHighGear() {
	    return m_periodicIO.isHighGear;
    }

	public synchronized final boolean isTrajectoryFinished() {
		if (Objects.isNull(m_motionPlanner) || m_controlMode != ControlMode.TRAJECTORY_TRACKING) {
			return true;
		} else {
			return m_motionPlanner.isDone() || m_forceFinishTrajectory;
		}
	}

	public synchronized void forceFinishTrajectory() {
		if (m_controlMode == ControlMode.TRAJECTORY_TRACKING && !Objects.isNull(m_motionPlanner)) {
			m_forceFinishTrajectory = true;
		}
	}

	public WheelState getWheelProperty(final Function<MotorController<MotorType>, Double> getter) {
		return new WheelState(getter.apply(m_masterMotorLeft), getter.apply(m_masterMotorRight));
	}

	public synchronized void setHeading(final Rotation heading) {
		m_headingOffset = heading.rotateBy(m_periodicIO.heading).inverse();
		// Update it here immediately; it's a single cycle in difference, but its something.
		m_periodicIO.heading = heading;
	}

	public synchronized Rotation getHeading() {
		return m_periodicIO.heading;
	}

	public synchronized final boolean isCollisionOccurring() {
	    return m_periodicIO.isCollisionOccurring;
    }

    public synchronized final boolean isTipping() {
	    return m_periodicIO.isTipping;
    }

	public synchronized DifferentialDriveModel.Dynamics getDynamics() {
        return m_model.solveForwardDynamics(
				m_model.forwardKinematics(m_periodicIO.velocityMeters),
				getWheelProperty(MotorController::getVoltageOutput),
				m_periodicIO.isHighGear);
	}

	public final FieldToVehicleEstimator getOdometer() {
	    return m_odometer;
    }

    public final TrajectoryGenerator getTrajectoryGenerator() {
	    return m_generator;
    }

    public synchronized final Optional<RigidTransform> getTrackingError() {
		return Optional.ofNullable(m_periodicIO.trackingError);
	}

    public synchronized final Optional<TimedState<RigidTransformWithCurvature>> getTrackingReference() {
	    return Optional.ofNullable(m_periodicIO.trackingReference);
    }

    public final DifferentialDriveModel getModel() {
	    return m_model;
    }
}
