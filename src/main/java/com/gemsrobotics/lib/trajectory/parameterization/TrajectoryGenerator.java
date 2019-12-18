package com.gemsrobotics.lib.trajectory.parameterization;

import com.gemsrobotics.lib.controls.DriveMotionPlanner;
import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDriveModel;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.RigidTransformWithCurvature;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.trajectory.DistanceView;
import com.gemsrobotics.lib.trajectory.Trajectory;
import com.gemsrobotics.lib.trajectory.TrajectoryContainer;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

public final class TrajectoryGenerator {
	private DriveMotionPlanner.MotionConfig m_config;
	private DifferentialDriveModel m_model;

	public TrajectoryGenerator(final DriveMotionPlanner.MotionConfig config, final DifferentialDriveModel model) {
		m_config = config;
		m_model = model;
	}

	public TrajectoryContainer<RigidTransformWithCurvature> generateTrajectory(
	        final boolean highGear,
			final boolean reversed,
			final List<RigidTransform> waypoints,
			final List<TimingConstraint<RigidTransformWithCurvature>> constraints
	) {
		return generateTrajectory(
		        highGear,
				reversed,
				waypoints,
				constraints,
				0.0,
				0.0
		);
	}

	public TrajectoryContainer<RigidTransformWithCurvature> generateTrajectory(
	        final boolean highGear,
			final boolean reversed,
			List<RigidTransform> waypoints,
			final List<TimingConstraint<RigidTransformWithCurvature>> constraints,
			final double velocityStart,
			final double velocityEnd
	) {
		final RigidTransform flip = RigidTransform.fromRotation(new Rotation(-1, 0, false));

		if (reversed) {
			waypoints = waypoints.stream().map(waypoint -> waypoint.transformBy(flip)).collect(Collectors.toList());
		}

		// Create a trajectory from splines.
        Trajectory<RigidTransformWithCurvature> trajectory = TrajectoryUtils.trajectoryFromSplineWaypoints(waypoints, m_config);

        if (reversed) {
			final List<RigidTransformWithCurvature> flipped = new ArrayList<>(trajectory.length());

			for (int i = 0; i < trajectory.length(); i++) {
				final var curvedState = new RigidTransformWithCurvature(
						trajectory.getState(i).getRigidTransform().transformBy(flip),
						-trajectory.getState(i).getCurvature(),
						trajectory.getState(i).getDCurvatureDs());

				flipped.add(curvedState);
			}

			trajectory = new Trajectory<>(flipped);
		}

		// Create the constraint that the robot must be able to traverse the trajectory without ever applying more
		// than the specified voltage.
		final DifferentialDriveDynamicsConstraint<RigidTransformWithCurvature> drivingConstraints =
				new	DifferentialDriveDynamicsConstraint<>(m_model, highGear, m_config.maxVoltage    );

        final CentripetalAccelerationConstraint centripetalAccelerationConstraints =
                new CentripetalAccelerationConstraint(m_config.maxCentripetalAcceleration);

        final List<TimingConstraint<RigidTransformWithCurvature>> fullConstraints = new ArrayList<>();
		fullConstraints.add(drivingConstraints);
		fullConstraints.add(centripetalAccelerationConstraints);

		if (!Objects.isNull(constraints)) {
			fullConstraints.addAll(constraints);
		}

		// Generate the timed trajectory.
		return new TrajectoryContainer<RigidTransformWithCurvature>(
		        highGear,
                Parameterizer.timeParameterizeTrajectory(
                    reversed,
                    new DistanceView<>(trajectory),
                    m_config.maxDx,
                    fullConstraints,
                    m_config,
                    velocityStart,
                    velocityEnd));
	}
}
