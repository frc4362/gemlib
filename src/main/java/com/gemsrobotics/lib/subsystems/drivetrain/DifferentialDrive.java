package com.gemsrobotics.lib.subsystems.drivetrain;

import com.gemsrobotics.lib.controls.DriveMotionPlanner;
import com.gemsrobotics.lib.controls.PIDFController;
import com.gemsrobotics.lib.drivers.imu.CollisionDetectingIMU;
import com.gemsrobotics.lib.drivers.imu.NavX;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.transmission.Transmission;
import com.gemsrobotics.lib.telemetry.monitoring.ConnectionMonitor;
import com.gemsrobotics.lib.telemetry.reporting.Reporter.Event.Kind;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.RigidTransformWithCurvature;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.physics.MotorTransmission;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.trajectory.TrajectoryIterator;
import com.gemsrobotics.lib.trajectory.parameterization.TimedState;
import com.gemsrobotics.lib.controls.DriveMotionPlanner.FollowerType;
import com.gemsrobotics.lib.trajectory.parameterization.TrajectoryGenerator;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Function;

@SuppressWarnings({"unused", "WeakerAccess", "OverlyCoupledClass"})
public abstract class DifferentialDrive extends Subsystem {
    public static class Config {
		public double rotationsToMeters;
		public double maxVoltage;

		public DriveMotionPlanner.MotionConfig motionConfig;
        public CollisionDetectingIMU.Thresholds collisionThresholds;

		public PIDFController.Gains gainsLowGear;
		public PIDFController.Gains gainsHighGear;

		public MotorTransmission.Properties propertiesHighGear;
		public MotorTransmission.Properties propertiesLowGear;
		public Model.Properties propertiesModel;

		public OpenLoopDriveHelper.Config openLoopConfig;
	}

	protected final Model m_model;
	protected final Config m_config;
	protected final OpenLoopDriveHelper m_openLoopHelper;
	protected transient final CollisionDetectingIMU m_imu;
	protected FieldToVehicleEstimator m_odometry;
	protected final DriveMotionPlanner m_motionPlanner;
    protected final TrajectoryGenerator m_generator;
    protected final PeriodicIO m_periodicIO;

	protected final MotorController m_masterLeft, m_masterRight, m_slaveLeft, m_slaveRight;
	protected final Transmission m_transmission;

	protected ControlMode m_controlState;
	protected boolean m_isHighGear, m_forceFinishPath;
	protected Rotation m_headingOffset;

    protected abstract Config getConfig();
    protected abstract List<MotorController> getMotorControllers();
    protected abstract Transmission getTransmission();

	protected DifferentialDrive() {
		m_config = getConfig();
		m_transmission = getTransmission();

		final var motors = getMotorControllers();
		m_masterLeft = motors.get(0);
		configureMotorController(m_masterLeft, true);
		m_slaveLeft = motors.get(1);
		m_slaveLeft.follow(m_masterLeft, false);
		m_masterRight = motors.get(2);
		configureMotorController(m_masterRight, false);
		m_slaveRight = motors.get(3);
		m_slaveRight.follow(m_masterRight, false);

		m_model = new Model(m_config.propertiesModel, new MotorTransmission(m_config.propertiesLowGear), new MotorTransmission(m_config.propertiesHighGear));

		m_motionPlanner = new DriveMotionPlanner(m_config.motionConfig, m_model, FollowerType.RAMSETE, this::isHighGear);
        m_generator = new TrajectoryGenerator(m_config.motionConfig, m_model);

		m_openLoopHelper = new OpenLoopDriveHelper(m_config.openLoopConfig);
        m_imu = new NavX();

        m_headingOffset = Rotation.identity();
		m_forceFinishPath = false;

        m_periodicIO = new PeriodicIO();
	}

	private void configureMotorController(final MotorController controller, final boolean isLeft) {
		controller.setInvertedOutput(!isLeft);
		controller.setRotationsPerMeter(m_config.rotationsToMeters);
		controller.setSelectedProfile(slotForGear(false));
		controller.setPIDF(m_config.gainsLowGear);
		controller.setSelectedProfile(slotForGear(true));
		controller.setPIDF(m_config.gainsHighGear);
	}

	public enum ControlMode {
		DISABLED,
		OPEN_LOOP,
		PATH_FOLLOWING
	}

	private class PeriodicIO implements Loggable {
		// INPUTS
        @Log.ToString(name="Wheel Position (m, m)")
		public Model.WheelState positionMeters = new Model.WheelState();
        @Log.ToString(name="Wheel Delta Position (m, m)")
		public Model.WheelState positionDeltaMeters = new Model.WheelState();
        @Log.ToString(name="Wheel Velocity (m per s, m per s)")
		public Model.WheelState velocityMeters = new Model.WheelState();
        @Log.ToString(name="Wheel Acceleration (m per s^2, m per s^2)")
		public Model.WheelState accelerationMeters = new Model.WheelState();
        @Log.ToString(name="Heading (deg)")
		public Rotation heading = Rotation.identity();
        @Log(name="High Gear? (Boolean)")
		public boolean isHighGear = DifferentialDrive.this.m_transmission.isHighGear();

        @Log(name="Tipping? (Boolean)")
		public boolean isTipping = false;
        @Log(name="Colliding? (Boolean)")
		public boolean isCollisionOccurring = false;

		// OUTPUTS
        @Log.ToString(name="Control Mode")
        public ControlMode demandType = null;
        @Log.ToString(name="Current Demand")
		public Model.WheelState demand = new Model.WheelState();
        @Log.ToString(name="Feedforward Demand")
		public Model.WheelState feedforward = new Model.WheelState();

		public RigidTransform error = new RigidTransform();
		public TimedState<RigidTransformWithCurvature> pathReference = null;
	}

	public synchronized void setNeutralBehaviour(final MotorController.NeutralBehaviour mode) {
		m_masterLeft.setNeutralBehaviour(mode);
		m_slaveLeft.setNeutralBehaviour(mode);
		m_masterRight.setNeutralBehaviour(mode);
		m_slaveRight.setNeutralBehaviour(mode);
	}

	public synchronized void setHighGear(final boolean wantsHighGear) {
		if (wantsHighGear != m_isHighGear) {
			m_isHighGear = wantsHighGear;
			m_transmission.setHighGear(wantsHighGear);

            final int profile = slotForGear(wantsHighGear);
            m_masterLeft.setSelectedProfile(profile);
            m_masterRight.setSelectedProfile(profile);
		}
	}

	public synchronized void setDisabled() {
		if (m_controlState != DifferentialDrive.ControlMode.DISABLED) {
			m_masterLeft.setNeutral();
			m_masterRight.setNeutral();
			m_controlState = DifferentialDrive.ControlMode.DISABLED;
            m_periodicIO.demand = new Model.WheelState();
            m_periodicIO.feedforward = new Model.WheelState();
            m_periodicIO.pathReference = null;
		}
	}

	public synchronized void setOpenLoop(final double throttle, final double wheel, final boolean isQuickTurn) {
		if (m_controlState != ControlMode.OPEN_LOOP) {
			setNeutralBehaviour(MotorController.NeutralBehaviour.BRAKE);
			m_controlState = ControlMode.OPEN_LOOP;
            m_periodicIO.feedforward = new Model.WheelState();
            m_periodicIO.pathReference = null;
		}

		m_periodicIO.demand = m_openLoopHelper.drive(throttle, wheel, isQuickTurn, m_periodicIO.isHighGear);
        m_periodicIO.feedforward = new Model.WheelState();
	}

	public synchronized void setTrajectory(final TrajectoryIterator<TimedState<RigidTransformWithCurvature>> trajectory) {
		if (!Objects.isNull(m_motionPlanner)) {
			m_forceFinishPath = false;
			m_motionPlanner.reset();
			m_motionPlanner.setTrajectory(trajectory);

			m_openLoopHelper.reset();
            m_controlState = ControlMode.PATH_FOLLOWING;
		}
	}

	protected final void driveOpenLoop(final Model.WheelState demand) {
		m_masterLeft.setDutyCycle(demand.left);
		m_masterRight.setDutyCycle(demand.right);
	}

	protected final void driveVelocity(final Model.WheelState demand, final Model.WheelState feedforward) {
		m_masterLeft.setVelocityMetersPerSecond(demand.left, feedforward.left);
		m_masterRight.setVelocityMetersPerSecond(demand.right, feedforward.right);
	}

	@Override
	protected synchronized void readPeriodicInputs() {
        m_periodicIO.demandType = m_controlState;

        // Position calculations
		final var oldPosition = new Model.WheelState(m_periodicIO.positionMeters);
		final var newPosition = getWheelProperty(MotorController::getPositionMeters);
		m_periodicIO.positionMeters = newPosition;
		m_periodicIO.positionDeltaMeters = newPosition.difference(oldPosition);

		// Velocity and acceleration calculations
		final var oldVelocity = new Model.WheelState(m_periodicIO.velocityMeters);
		final var newVelocity = getWheelProperty(MotorController::getVelocityLinearMetersPerSecond);
		m_periodicIO.velocityMeters = newVelocity;
		m_periodicIO.accelerationMeters = newVelocity.difference(oldVelocity).map(val -> val / dt());

		// IMU reads and adjustments
		m_periodicIO.heading = m_imu.getYaw().rotateBy(m_headingOffset);
        m_periodicIO.isCollisionOccurring = m_imu.isCollisionOccurring();
        m_periodicIO.isTipping = m_imu.isTipping();

        m_periodicIO.isHighGear = m_transmission.isHighGear();
	}

	protected void updateStateEstimation(final double timestamp) {
		final var measuredVelocity = m_odometry.generateOdometryFromSensors(m_periodicIO.positionDeltaMeters, m_periodicIO.heading);
		final var predictedVelocity = m_model.forwardKinematics(m_periodicIO.velocityMeters.left, m_periodicIO.velocityMeters.right);

		m_odometry.addObservations(timestamp, measuredVelocity, predictedVelocity);
	}

	protected void updateTrajectoryFollowingDemands(final double timestamp) {
		if (m_controlState == ControlMode.PATH_FOLLOWING) {
			final var output = m_motionPlanner.update(timestamp, m_odometry.getFieldToVehicle(timestamp));

			m_periodicIO.error = m_motionPlanner.getError();
			m_periodicIO.pathReference = m_motionPlanner.getSetpoint();

			output.ifPresent(outputs -> {
			   if (!m_forceFinishPath) {
                   // convert from radians/second to meters/second
                   // please note that we cannot use setVelocityRPM as the calculated rad/s are for the wheel, not the drive shaft
                   m_periodicIO.demand = outputs.velocity.map(radiansPerSecond -> (m_config.propertiesModel.wheelbaseRadius * radiansPerSecond));
                   // convert from volts to % output
                   m_periodicIO.feedforward = outputs.feedforwardVoltage.map(v -> v / 12.0);
               }
            });
		} else {
			report(Kind.ERROR, "Tried to update trajectory following in bad control state: " + m_controlState.toString());
		}
	}

	@Override
	protected synchronized void onCreate(final double timestamp) {
        m_odometry = FieldToVehicleEstimator.withStarting(m_model, timestamp, RigidTransform.identity());
        setDisabled();
        setHighGear(false);
	}

	@Override
	protected synchronized void onEnable(final double timestamp) {
        setOpenLoop(0, 0, false);
	}

	@Override
	protected synchronized void onUpdate(final double timestamp) {
        updateStateEstimation(timestamp);

        switch (m_controlState) {
            case DISABLED:
                // No actuation
                break;
            case OPEN_LOOP:
                // This makes sense, since this demand was updated externally
                driveOpenLoop(m_periodicIO.demand);
                break;
            case PATH_FOLLOWING:
                // This makes sense, since the demands are updated in this method
                updateTrajectoryFollowingDemands(timestamp);
                driveVelocity(m_periodicIO.demand, m_periodicIO.feedforward);
                break;
        }
	}

	@Override
	protected synchronized void onStop(final double timestamp) {
		setDisabled();
		setNeutralBehaviour(ConnectionMonitor.getInstance().hasConnectedToField()
                ? MotorController.NeutralBehaviour.BRAKE
                : MotorController.NeutralBehaviour.COAST);
	}

	@Override
	public void setSafeState() {
		setDisabled();
		setNeutralBehaviour(MotorController.NeutralBehaviour.COAST);
	}

	@Override
	public FaultedResponse checkFaulted() {
	    if (!m_masterLeft.isEncoderPresent()) {
	        report(Kind.HARDWARE_FAULT, "Left encoder lost!");
        }

	    if (!m_masterRight.isEncoderPresent()) {
	        report(Kind.HARDWARE_FAULT, "Right encoder lost!");
        }

	    return FaultedResponse.NONE;
	}

    private int slotForGear(final boolean isHighGear) {
	    return isHighGear ? 1 : 0;
    }

	public synchronized final boolean isHighGear() {
	    return m_periodicIO.isHighGear;
    }

	public synchronized final boolean isTrajectoryFinished() {
		if (Objects.isNull(m_motionPlanner) || m_controlState != ControlMode.PATH_FOLLOWING) {
			return false;
		} else {
			return m_motionPlanner.isDone() || m_forceFinishPath;
		}
	}

	public synchronized void forceFinishTrajectory() {
		if (m_controlState == ControlMode.PATH_FOLLOWING && !Objects.isNull(m_motionPlanner)) {
			m_forceFinishPath = true;
		}
	}

	public Model.WheelState getWheelProperty(final Function<MotorController, Double> getter) {
		return new Model.WheelState(getter.apply(m_masterLeft), getter.apply(m_masterRight));
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

	public synchronized Model.Dynamics getDynamics() {
        return m_model.solveInverseDynamics(m_periodicIO.velocityMeters, m_periodicIO.accelerationMeters, m_isHighGear);
	}

	public final FieldToVehicleEstimator getOdometer() {
	    return m_odometry;
    }

    public final TrajectoryGenerator getTrajectoryGenerator() {
	    return m_generator;
    }

    public synchronized final Optional<TimedState<RigidTransformWithCurvature>> getPathReference() {
	    return Optional.ofNullable(m_periodicIO.pathReference);
    }

    public final Model getModelling() {
	    return m_model;
    }
}
