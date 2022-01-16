package com.gemsrobotics.lib.drivers.motorcontrol;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DriverStation;

public final class MotorControllerFactory {
    private static final int TIMEOUT_MS = 100;

    private MotorControllerFactory() { }

    public static class SparkConfiguration {
        public boolean BURN_FACTORY_DEFAULT_FLASH;
        public IdleMode IDLE_MODE;
        public boolean INVERTED;

        public int STATUS_FRAME_0_RATE_MS;
        public int STATUS_FRAME_1_RATE_MS;
        public int STATUS_FRAME_2_RATE_MS;

        public double OPEN_LOOP_RAMP_RATE;
        public double CLOSED_LOOP_RAMP_RATE;

        public boolean ENABLE_VOLTAGE_COMPENSATION;
        public double NOMINAL_VOLTAGE;
    }

    public static final SparkConfiguration DEFAULT_SPARK_CONFIG = new SparkConfiguration() {
        {
             BURN_FACTORY_DEFAULT_FLASH = false;
             IDLE_MODE = IdleMode.kCoast;
             INVERTED = false;
             STATUS_FRAME_0_RATE_MS = 10;
             STATUS_FRAME_1_RATE_MS = 1000;
             STATUS_FRAME_2_RATE_MS = 1000;
             OPEN_LOOP_RAMP_RATE = 0.0;
             CLOSED_LOOP_RAMP_RATE = 0.0;
             ENABLE_VOLTAGE_COMPENSATION = false;
             NOMINAL_VOLTAGE = 12.0;
        }
    };

    private static final SparkConfiguration SLAVE_SPARK_CONFIG = new SparkConfiguration() {
        {
            BURN_FACTORY_DEFAULT_FLASH = false;
            IDLE_MODE = IdleMode.kCoast;
            INVERTED = false;
            STATUS_FRAME_0_RATE_MS = 1000;
            STATUS_FRAME_1_RATE_MS = 1000;
            STATUS_FRAME_2_RATE_MS = 1000;
            OPEN_LOOP_RAMP_RATE = 0.0;
            CLOSED_LOOP_RAMP_RATE = 0.0;
            ENABLE_VOLTAGE_COMPENSATION = false;
            NOMINAL_VOLTAGE = 12.0;
        }
    };

    private static void handleCANError(final int id, final REVLibError error, final String message) {
        if (error != REVLibError.kOk) {
            DriverStation.reportError("Could not configure spark id: " + id + " error: " + error.toString() + " " + message, false);
        }
    }

    public static GemSparkMax createDefaultSparkMax(final int port) {
        return createSparkMax(port, DEFAULT_SPARK_CONFIG);
    }

    public static GemSparkMax createSparkMax(final int port, final SparkConfiguration config) {
        final CANSparkMax sparkMax = new CANSparkMax(port, CANSparkMaxLowLevel.MotorType.kBrushless);

        sparkMax.setCANTimeout(200);

        handleCANError(port, sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, config.STATUS_FRAME_0_RATE_MS), "set status0 rate");
        handleCANError(port, sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, config.STATUS_FRAME_1_RATE_MS), "set status1 rate");
        handleCANError(port, sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, config.STATUS_FRAME_2_RATE_MS), "set status2 rate");

        handleCANError(port, sparkMax.setIdleMode(config.IDLE_MODE), "set neutral");
        sparkMax.setInverted(config.INVERTED);
        handleCANError(port, sparkMax.setOpenLoopRampRate(config.OPEN_LOOP_RAMP_RATE), "set open loop ramp");
        handleCANError(port, sparkMax.setClosedLoopRampRate(config.CLOSED_LOOP_RAMP_RATE), "set closed loop ramp");

        if (config.ENABLE_VOLTAGE_COMPENSATION) {
            handleCANError(port, sparkMax.enableVoltageCompensation(config.NOMINAL_VOLTAGE), "voltage compensation");
        } else {
            handleCANError(port, sparkMax.disableVoltageCompensation(), "voltage compensation");
        }

        return new GemSparkMax(sparkMax);
    }

    public static class TalonConfiguration {
        public NeutralMode NEUTRAL_MODE;
        // factory default
        public double NEUTRAL_DEADBAND;

        public boolean ENABLE_CURRENT_LIMIT;
        public boolean ENABLE_SOFT_LIMIT;
        public boolean ENABLE_LIMIT_SWITCH;
        public int FORWARD_SOFT_LIMIT;
        public int REVERSE_SOFT_LIMIT;

        public boolean INVERTED;
        public boolean SENSOR_PHASE;

        public int CONTROL_FRAME_PERIOD_MS;
        public int MOTION_CONTROL_FRAME_PERIOD_MS;
        public int GENERAL_STATUS_FRAME_RATE_MS;
        public int FEEDBACK_STATUS_FRAME_RATE_MS;
        public int QUAD_ENCODER_STATUS_FRAME_RATE_MS;
        public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS;
        public int PULSE_WIDTH_STATUS_FRAME_RATE_MS;

        public SensorVelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD;
        public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW;

        public double OPEN_LOOP_RAMP_RATE;
        public double CLOSED_LOOP_RAMP_RATE;
    }

    private static final TalonConfiguration DEFAULT_TALON_CONFIG = new TalonConfiguration() {
        {
             NEUTRAL_MODE = NeutralMode.Coast;
             NEUTRAL_DEADBAND = 0.04;

             ENABLE_CURRENT_LIMIT = false;
             ENABLE_SOFT_LIMIT = false;
             ENABLE_LIMIT_SWITCH = false;
             FORWARD_SOFT_LIMIT = 0;
             REVERSE_SOFT_LIMIT = 0;

             INVERTED = false;
             SENSOR_PHASE = false;

             CONTROL_FRAME_PERIOD_MS = 5;
             MOTION_CONTROL_FRAME_PERIOD_MS = 100;
             GENERAL_STATUS_FRAME_RATE_MS = 5;
             FEEDBACK_STATUS_FRAME_RATE_MS = 10;
             QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
             ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
             PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;

             VELOCITY_MEASUREMENT_PERIOD = SensorVelocityMeasPeriod.Period_10Ms;
             VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 32;

             OPEN_LOOP_RAMP_RATE = 0.0;
             CLOSED_LOOP_RAMP_RATE = 0.0;
        }
    };

    private static final TalonConfiguration HIGH_PERFORMANCE_TALON_CONFIG = new TalonConfiguration() {
        {
            NEUTRAL_MODE = NeutralMode.Coast;
            NEUTRAL_DEADBAND = 0.02;

            ENABLE_CURRENT_LIMIT = false;
            ENABLE_SOFT_LIMIT = false;
            ENABLE_LIMIT_SWITCH = false;
            FORWARD_SOFT_LIMIT = 0;
            REVERSE_SOFT_LIMIT = 0;

            INVERTED = false;
            SENSOR_PHASE = false;

            CONTROL_FRAME_PERIOD_MS = 5;
            MOTION_CONTROL_FRAME_PERIOD_MS = 100;
            GENERAL_STATUS_FRAME_RATE_MS = 2;
            FEEDBACK_STATUS_FRAME_RATE_MS = 10;
            QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
            ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 2;
            PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;

            VELOCITY_MEASUREMENT_PERIOD = SensorVelocityMeasPeriod.Period_10Ms;
            VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 32;

            OPEN_LOOP_RAMP_RATE = 0.0;
            CLOSED_LOOP_RAMP_RATE = 0.0;
        }
    };

    private static final TalonConfiguration SLAVE_TALON_CONFIG = new TalonConfiguration() {
        {
            NEUTRAL_MODE = NeutralMode.Coast;
            NEUTRAL_DEADBAND = 0.04;

            ENABLE_CURRENT_LIMIT = false;
            ENABLE_SOFT_LIMIT = false;
            ENABLE_LIMIT_SWITCH = false;
            FORWARD_SOFT_LIMIT = 0;
            REVERSE_SOFT_LIMIT = 0;

            INVERTED = false;
            SENSOR_PHASE = false;

            CONTROL_FRAME_PERIOD_MS = 100;
            MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
            GENERAL_STATUS_FRAME_RATE_MS = 1000;
            FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
            QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
            ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
            PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;

            VELOCITY_MEASUREMENT_PERIOD = SensorVelocityMeasPeriod.Period_100Ms;
            VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

            OPEN_LOOP_RAMP_RATE = 0.0;
            CLOSED_LOOP_RAMP_RATE = 0.0;
        }
    };

    private static void configureTalon(final BaseTalon talon, final TalonConfiguration config) {
        talon.set(ControlMode.PercentOutput, 0.0);

        talon.clearStickyFaults(TIMEOUT_MS);

        talon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, TIMEOUT_MS);
        talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, TIMEOUT_MS);
        talon.overrideLimitSwitchesEnable(config.ENABLE_LIMIT_SWITCH);

        // Turn off re:zeroing by default.
        talon.configSetParameter(ParamEnum.eClearPositionOnLimitF, 0, 0, 0, TIMEOUT_MS);
        talon.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, TIMEOUT_MS);

        talon.configNominalOutputForward(0, TIMEOUT_MS);
        talon.configNominalOutputReverse(0, TIMEOUT_MS);
        talon.configNeutralDeadband(config.NEUTRAL_DEADBAND, TIMEOUT_MS);

        talon.configPeakOutputForward(1.0, TIMEOUT_MS);
        talon.configPeakOutputReverse(-1.0, TIMEOUT_MS);

        talon.setNeutralMode(config.NEUTRAL_MODE);

        talon.configForwardSoftLimitThreshold(config.FORWARD_SOFT_LIMIT, TIMEOUT_MS);
        talon.configForwardSoftLimitEnable(config.ENABLE_SOFT_LIMIT, TIMEOUT_MS);

        talon.configReverseSoftLimitThreshold(config.REVERSE_SOFT_LIMIT, TIMEOUT_MS);
        talon.configReverseSoftLimitEnable(config.ENABLE_SOFT_LIMIT, TIMEOUT_MS);
        talon.overrideSoftLimitsEnable(config.ENABLE_SOFT_LIMIT);

        talon.setInverted(config.INVERTED);
        talon.setSensorPhase(config.SENSOR_PHASE);

        talon.selectProfileSlot(0, 0);

        talon.configVelocityMeasurementPeriod(config.VELOCITY_MEASUREMENT_PERIOD, TIMEOUT_MS);
        talon.configVelocityMeasurementWindow(config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW, TIMEOUT_MS);

        talon.configOpenloopRamp(config.OPEN_LOOP_RAMP_RATE, TIMEOUT_MS);
        talon.configClosedloopRamp(config.CLOSED_LOOP_RAMP_RATE, TIMEOUT_MS);

        talon.enableVoltageCompensation(false);
        talon.configVoltageCompSaturation(0.0, TIMEOUT_MS);
        talon.configVoltageMeasurementFilter(32, TIMEOUT_MS);

        if (talon instanceof TalonSRX) {
            talon.changeMotionControlFramePeriod(config.MOTION_CONTROL_FRAME_PERIOD_MS);
            talon.clearMotionProfileHasUnderrun(TIMEOUT_MS);
            talon.clearMotionProfileTrajectories();

            ((TalonSRX) talon).enableCurrentLimit(false);
        }

        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, config.GENERAL_STATUS_FRAME_RATE_MS, TIMEOUT_MS);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, config.FEEDBACK_STATUS_FRAME_RATE_MS, TIMEOUT_MS);

        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, config.QUAD_ENCODER_STATUS_FRAME_RATE_MS, TIMEOUT_MS);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, config.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS, TIMEOUT_MS);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, config.PULSE_WIDTH_STATUS_FRAME_RATE_MS, TIMEOUT_MS);

        talon.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS);
    }

    private static GemTalon<TalonSRX> createTalonSRX(final int port, final TalonConfiguration config, final boolean isSlave) {
        final var talon = new TalonSRX(port);
        configureTalon(talon, config);
        return new GemTalon<>(talon, isSlave);
    }

    public static GemTalon<TalonSRX> createDefaultTalonSRX(final int port) {
        return createTalonSRX(port, DEFAULT_TALON_CONFIG, false);
    }

    public static GemTalon<TalonSRX> createSlaveTalonSRX(final int port) {
        return createTalonSRX(port, SLAVE_TALON_CONFIG, true);
    }

    public static GemTalon<TalonFX> createTalonFX(final int port, final TalonConfiguration config, final boolean isSlave) {
        final var talon = new TalonFX(port);
        configureTalon(talon, config);
        talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, TIMEOUT_MS);
        return new GemTalon<>(talon, isSlave);
    }

    public static GemTalon<TalonFX> createDefaultTalonFX(final int port) {
        return createTalonFX(port, DEFAULT_TALON_CONFIG, false);
    }

    public static GemTalon<TalonFX> createSlaveTalonFX(final int port) {
        return createTalonFX(port, SLAVE_TALON_CONFIG, true);
    }
}
