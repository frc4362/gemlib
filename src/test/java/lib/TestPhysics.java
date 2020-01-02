package lib;

import com.gemsrobotics.lib.physics.MotorModel;
import com.gemsrobotics.lib.subsystems.drivetrain.DifferentialDriveModel;
import com.gemsrobotics.lib.subsystems.drivetrain.WheelState;

import com.gemsrobotics.lib.subsystems.drivetrain.ChassisState;

import com.gemsrobotics.lib.utils.MathUtils;
import com.gemsrobotics.lib.utils.Units;
import org.junit.Test;

import static com.gemsrobotics.lib.utils.MathUtils.epsilonEquals;
import static com.gemsrobotics.lib.utils.MathUtils.Bounds;
import static org.hamcrest.MatcherAssert.assertThat;
import static org.hamcrest.Matchers.*;

public class TestPhysics {
    @Test
    public void testMotorTransmission() {
        final double kEpsilon = MathUtils.Epsilon;
        // 100 rpm per V, .2 N*m per V, 1.5V to overcome friction.
        final var motor = new MotorModel(new MotorModel.Properties() {{
            speedRadiansPerSecondPerVolt = Units.rpm2RadsPerSecond(100.0);
            torquePerVolt = 0.2;
            stictionVoltage = 1.5;
        }});

        assertThat(Units.rpm2RadsPerSecond(1200.0), closeTo(motor.freeSpeedAtVoltageRadiansPerSecond(12.0 + 1.5), kEpsilon));
        assertThat(Units.rpm2RadsPerSecond(600.0), closeTo(motor.freeSpeedAtVoltageRadiansPerSecond(6.0 + 1.5), kEpsilon));
        assertThat(Units.rpm2RadsPerSecond(0.0), closeTo(motor.freeSpeedAtVoltageRadiansPerSecond(1.4), kEpsilon));
        assertThat(Units.rpm2RadsPerSecond(0.0), closeTo(motor.freeSpeedAtVoltageRadiansPerSecond(0.0), kEpsilon));
        assertThat(Units.rpm2RadsPerSecond(0.0), closeTo(motor.freeSpeedAtVoltageRadiansPerSecond(-1.4), kEpsilon));
        assertThat(Units.rpm2RadsPerSecond(-600.0), closeTo(motor.freeSpeedAtVoltageRadiansPerSecond(-6.0 - 1.5), kEpsilon));
        assertThat(Units.rpm2RadsPerSecond(-1200.0), closeTo(motor.freeSpeedAtVoltageRadiansPerSecond(-12.0 - 1.5), kEpsilon));

        assertThat(.2 * 10.5, closeTo(motor.torqueForVoltage(0.0, 12.0), kEpsilon));
        assertThat(.2 * 3.5, closeTo(motor.torqueForVoltage(0.0, 5.0), kEpsilon));
        assertThat(-.2 * 10.5, closeTo(motor.torqueForVoltage(0.0, -12.0), kEpsilon));
        assertThat(0.0, closeTo(motor.torqueForVoltage(0.0, 0.0), kEpsilon));
        assertThat(0.0, closeTo(motor.torqueForVoltage(0.0, 1.4), kEpsilon));
        assertThat(0.0, closeTo(motor.torqueForVoltage(0.0, -1.4), kEpsilon));
        assertThat(0.0, closeTo(motor.torqueForVoltage(Units.rpm2RadsPerSecond(1200.0), 13.5), kEpsilon));
        assertThat(-.2 * 1.5, closeTo(motor.torqueForVoltage(Units.rpm2RadsPerSecond(1200.0), 12.0), kEpsilon));
        assertThat(.2 * 1.5, closeTo(motor.torqueForVoltage(Units.rpm2RadsPerSecond(1200.0), 15.0), kEpsilon));
        assertThat(0.0, closeTo(motor.voltageFromTorque(0.0, 0.0), kEpsilon));
        assertThat(13.5, closeTo(motor.voltageFromTorque(Units.rpm2RadsPerSecond(1200.0), 0.0), kEpsilon));
        assertThat(-13.5, closeTo(motor.voltageFromTorque(Units.rpm2RadsPerSecond(-1200.0), 0.0), kEpsilon));

        for (final double speed : new double[]{0.0, 0.1, -0.5, 130.1, 3000.0, -45.0, 666.666}) {
            for (final double voltage : new double[]{0.0, 1.0, 3.34, 6.4, -2.0, 0.9, -5.6, 12.1, 1.499, 1.501}) {
                double torque = motor.torqueForVoltage(Units.rpm2RadsPerSecond(speed), voltage);
                if (Math.abs(voltage) <= 1.5 && epsilonEquals(speed, 0.0)) {
                    assertThat(0.0, closeTo(torque, kEpsilon));
                } else {
                    assertThat(voltage, closeTo(motor.voltageFromTorque(Units.rpm2RadsPerSecond(speed), torque), kEpsilon));
                }
            }
        }
    }

    @Test
    public void testModel() {
        final double kEpsilon = 1;

        MotorModel transmission = new MotorModel(new MotorModel.Properties() {{
            speedRadiansPerSecondPerVolt = Units.rpm2RadsPerSecond(65.0);
            torquePerVolt = 0.35;
            stictionVoltage = 1.0;
        }});

        final var props = new DifferentialDriveModel.Properties() {
            {
                massKg = 70;
                angularMomentInertiaKgMetersSquared = 84;
                angularDragTorquePerRadiansPerSecond = 12.0;
                wheelRadiusMeters = Units.inches2Meters(2.0);
                wheelbaseRadiusMeters = Units.inches2Meters(25.0) / 2.0;
            }
        };

        DifferentialDriveModel drive = new DifferentialDriveModel(props, transmission);
//        // Kinematics
        ChassisState v1 = drive.forwardKinematics(new WheelState(0.0, 0.0));
        assertThat(0.0, closeTo(v1.linearMeters, kEpsilon));
        assertThat(0.0, closeTo(v1.angularRadians, kEpsilon));

        WheelState w1 = drive.inverseKinematics(v1);
        assertThat(0.0, closeTo(w1.left, kEpsilon));
        assertThat(0.0, closeTo(w1.right, kEpsilon));

        ChassisState v2 = drive.forwardKinematics(new WheelState(Units.rpm2RadsPerSecond(65.0 * 10.0), Units.rpm2RadsPerSecond(65.0 * 10.0)));
        assertThat(11.0, closeTo(Units.meters2Feet(v2.linearMeters), kEpsilon));
        assertThat(0.0, is(v2.angularRadians));

        WheelState w2 = drive.inverseKinematics(v2);
        assertThat(Units.rpm2RadsPerSecond(65.0 * 10.0), is(w2.left));
        assertThat(Units.rpm2RadsPerSecond(65.0 * 10.0), is(w2.right));

        ChassisState v3 = drive.forwardKinematics(new WheelState(Units.rpm2RadsPerSecond(65.0 * -10.0), Units.rpm2RadsPerSecond(65.0 * -10.0)));
        assertThat(-11.0, closeTo(Units.meters2Feet(v3.linearMeters), 1.0));
        assertThat(0.0, is(v3.angularRadians));

        WheelState w3 = drive.inverseKinematics(v3);
        assertThat(Units.rpm2RadsPerSecond(-65.0 * 10.0), closeTo(w3.left, kEpsilon));
        assertThat(Units.rpm2RadsPerSecond(-65.0 * 10.0), closeTo(w3.right, kEpsilon));

        ChassisState v4 = drive.forwardKinematics(new WheelState(Units.rpm2RadsPerSecond(-65.0 * 10.0), Units.rpm2RadsPerSecond(65.0 * 10.0)));
        assertThat(0.0, closeTo(Units.meters2Feet(v4.linearMeters), kEpsilon));
        assertThat(10.0, closeTo(v4.angularRadians, kEpsilon));

        WheelState w4 = drive.inverseKinematics(v4);
        assertThat(Units.rpm2RadsPerSecond(-65.0 * 10.0), closeTo(w4.left, kEpsilon));
        assertThat(Units.rpm2RadsPerSecond(65.0 * 10.0), closeTo(w4.right, kEpsilon));

        ChassisState v5 = drive.forwardKinematics(new WheelState(Units.rpm2RadsPerSecond(65.0 * 5.0), Units.rpm2RadsPerSecond(-65.0 * 5.0)));
        assertThat(0.0, closeTo(Units.meters2Feet(v5.linearMeters), kEpsilon));
        assertThat(-5.0, closeTo(v5.angularRadians, kEpsilon));

        WheelState w5 = drive.inverseKinematics(v5);
        assertThat(Units.rpm2RadsPerSecond(65.0 * 5.0), closeTo(w5.left, kEpsilon));
        assertThat(Units.rpm2RadsPerSecond(-65.0 * 5.0), closeTo(w5.right, kEpsilon));

        // Forward dynamics.
        DifferentialDriveModel.Dynamics d1 = drive.solveForwardDynamics(new ChassisState(0.0, 0.0), new WheelState(0.0, 0.0), false);
        assertThat(0.0, closeTo(d1.torque.left, kEpsilon));
        assertThat(0.0, closeTo(d1.torque.right, kEpsilon));
        assertThat(0.0, closeTo(d1.wheelAccelerationRadiansPerSecondSquared.left, kEpsilon));
        assertThat(0.0, closeTo(d1.wheelAccelerationRadiansPerSecondSquared.right, kEpsilon));
        assertThat(0.0, closeTo(d1.chassisAcceleration.linearMeters, kEpsilon));
        assertThat(0.0, closeTo(d1.chassisAcceleration.angularRadians, kEpsilon));

        DifferentialDriveModel.Dynamics d2 = drive.solveForwardDynamics(new ChassisState(0.0, 0.0), new WheelState(12.0, 12.0), false);
        assertThat(11.0 * .35, closeTo(d2.torque.left, kEpsilon));
        assertThat(11.0 * .35, closeTo(d2.torque.right, kEpsilon));
        assertThat(0.0, lessThan(d2.wheelAccelerationRadiansPerSecondSquared.left));
        assertThat(0.0, lessThan(d2.wheelAccelerationRadiansPerSecondSquared.right));
        assertThat(2.0, lessThan(d2.chassisAcceleration.linearMeters));
        assertThat(0.0, closeTo(d2.chassisAcceleration.angularRadians, kEpsilon));

        DifferentialDriveModel.Dynamics d3 = drive.solveForwardDynamics(new ChassisState(0.0, 0.0), new WheelState(-12.0, -12.0), false);
        assertThat(-11.0 * .35, closeTo(d3.torque.left, kEpsilon));
        assertThat(-11.0 * .35, closeTo(d3.torque.right, kEpsilon));
        assertThat(0.0, greaterThan(d3.wheelAccelerationRadiansPerSecondSquared.left));
        assertThat(0.0, greaterThan(d3.wheelAccelerationRadiansPerSecondSquared.right));
        assertThat(0.0, greaterThan(d3.chassisAcceleration.linearMeters));
        assertThat(0.0, closeTo(d3.chassisAcceleration.angularRadians, kEpsilon));

        DifferentialDriveModel.Dynamics d4 = drive.solveForwardDynamics(new ChassisState(0.0, 0.0), new WheelState(-12.0, 12.0), false);
        assertThat(-11.0 * .35, closeTo(d4.torque.left, kEpsilon));
        assertThat(11.0 * .35, closeTo(d4.torque.right, kEpsilon));
        assertThat(0.0, greaterThan(d4.wheelAccelerationRadiansPerSecondSquared.left));
        assertThat(0.0, lessThan(d4.wheelAccelerationRadiansPerSecondSquared.right));
        assertThat(0.0, closeTo(d4.chassisAcceleration.linearMeters, kEpsilon));
        assertThat(0.0, lessThan(d4.chassisAcceleration.angularRadians));

        // Inverse dynamics.
        DifferentialDriveModel.Dynamics d5 = drive.solveInverseDynamics(new ChassisState(0.0, 0.0), new ChassisState(0.0, 0.0), false);
        assertThat(0.0, is(d5.torque.left));
        assertThat(0.0, is(d5.torque.right));
        assertThat(0.0, is(d5.voltage.left));
        assertThat(0.0, is(d5.voltage.right));

        DifferentialDriveModel.Dynamics d6 = drive.solveInverseDynamics(new ChassisState(Units.feet2Meters(10.0), 0.0), new ChassisState(0.0, 0.0), false);
        assertThat(0.0, closeTo(d6.torque.left, kEpsilon));
        assertThat(0.0, closeTo(d6.torque.right, kEpsilon));
        assertThat(9.5, closeTo(d6.voltage.left, kEpsilon));
        assertThat(9.5, closeTo(d6.voltage.right, kEpsilon));

        DifferentialDriveModel.Dynamics d7 = drive.solveInverseDynamics(new ChassisState(Units.inches2Meters(10.0 * 12), 0.0), new ChassisState(Units.inches2Meters(2.0 * 12.0), 0.0), false);
        assertThat(1.0, closeTo(d7.torque.left, kEpsilon));
        assertThat(1.0, closeTo(d7.torque.right, kEpsilon));
        assertThat(13.0, closeTo(d7.voltage.left, kEpsilon));
        assertThat(13.0, closeTo(d7.voltage.right, kEpsilon));

        DifferentialDriveModel.Dynamics d8 = drive.solveInverseDynamics(new ChassisState(Units.inches2Meters(10.0 * 12), 0.0), new ChassisState(Units.inches2Meters(-2.0 * 12), 0.0), false);
        assertThat(-1.0, closeTo(d8.torque.left, kEpsilon));
        assertThat(-1.0, closeTo(d8.torque.right, kEpsilon));
        assertThat(6.5, closeTo(d8.voltage.left, kEpsilon));
        assertThat(6.5, closeTo(d8.voltage.right, kEpsilon));

//        Model.Dynamics d9 = drive.solveInverseDynamics(new Model.ChassisState(Units.inches2Meters(10.0 * 12), Units.degrees2Rads(45.0)), new Model.ChassisState(Units.inches2Meters(2.0 * 12), Units.degrees2Rads(9.0)), false);
//        assertThat(1.0, closeTo(d9.torque.left, kEpsilon));
//        assertThat(1.0, closeTo(d9.torque.right, kEpsilon));
//        assertThat(11.0, closeTo(d9.voltage.left, kEpsilon));
//        assertThat(14.0, closeTo(d9.voltage.right, kEpsilon));

        // Max speed.
        assertThat(Units.feet2Meters(13.0), closeTo(drive.calculateMaxVelocity(0.0, 12.0, false), kEpsilon));
        assertThat(Units.feet2Meters(6.0), closeTo(drive.calculateMaxVelocity(0.0, 6.0, false), kEpsilon));
        assertThat(Units.feet2Meters(3.0), closeTo(drive.calculateMaxVelocity(1.0 / drive.wheelBaseRadiusMeters, 6.0, false), kEpsilon));
        assertThat(Units.feet2Meters(3.0), closeTo(drive.calculateMaxVelocity(-1.0 / drive.wheelBaseRadiusMeters, 6.0, false), kEpsilon));
        assertThat(Units.rpm2RadsPerSecond(50.0), closeTo(drive.calculateMaxVelocity(Double.POSITIVE_INFINITY, 6.0, false), kEpsilon));
        assertThat(Units.rpm2RadsPerSecond(-50.0), closeTo(drive.calculateMaxVelocity(Double.NEGATIVE_INFINITY, 6.0, false), kEpsilon));

        // Max acceleration.
        Bounds accelerations1 = drive.calculateMinMaxAcceleration(new ChassisState(0.0, 0.0), 0.0, 12.0, false);
        assertThat(2.0, closeTo(accelerations1.max, kEpsilon));
        assertThat(-2.0, closeTo(accelerations1.min, kEpsilon));

        Bounds accelerations2 = drive.calculateMinMaxAcceleration(new ChassisState(0.0, 0.0), 0.0, 6.0, false);
        assertThat(1.0, closeTo(accelerations2.max, kEpsilon));
        assertThat(-1.0, closeTo(accelerations2.min, kEpsilon));

        Bounds accelerations3 = drive.calculateMinMaxAcceleration(new ChassisState(Units.feet2Meters(8.0), 0.0), 0.0, 12.0, false);
        assertThat(1.0, closeTo(accelerations3.max, kEpsilon));
        assertThat(-4.0, closeTo(accelerations3.min, kEpsilon));

        Bounds accelerations4 = drive.calculateMinMaxAcceleration(new ChassisState(0.0, 0.0), Double.POSITIVE_INFINITY, 6.0, false);
        assertThat(1.0, closeTo(accelerations4.max, kEpsilon));
        assertThat(-1.0, closeTo(accelerations4.min, kEpsilon));
    }
}
