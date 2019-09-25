package com.gemsrobotics.lib.drivers.motorcontrol;

import com.gemsrobotics.lib.controls.PIDFController;
import com.gemsrobotics.lib.utils.Units;

public interface MotorController {
    class MotionParameters {
        double acceleration, cruiseVelocity, allowableError;

        public MotionParameters(final double maxAcceleration, final double cruiseVelocity, final double allowableError) {
            this.acceleration = maxAcceleration;
            this.cruiseVelocity = cruiseVelocity;
            this.allowableError = allowableError;
        }
    }

    enum NeutralBehaviour {
        COAST, BRAKE
    }

    /**
     * @return Volts fed into the motor controller
     */
    double getVoltageInput();
    /**
     * @return Volts applied to the motor
     */
    double getVoltageOutput();
    /**
     * @return Amps of the output
     */
    double getDrawnCurrent();

    /**
     * @return Whether or not an encoder is attached
     */
    boolean isEncoderPresent();

    /**
     * @param profileID Set what profile is chosen for configuration or use
     */
    void setSelectedProfile(int profileID);

    /**
     * @param other A motorcontroller of the same type as this one
     * @param invert Whether or not to reverse the output direction of the followed motor
     * @return If the operation was successful
     */
    boolean follow(MotorController other, boolean invert);

    /**
     * @param currentLimitAmps The allowed sustained current in amps of the motor controller
     * @return If the operation was successful
     */
    boolean setCurrentLimit(int currentLimitAmps);

    /**
     * @param inverted Should the output of the motor be reversed from what is commanded
     * @return If the operation was successful
     */
    boolean setInvertedOutput(boolean inverted);

    /**
     * @param mode The behaviour of the motor controller with no volts applied
     * @return If the operation was successful
     */
    boolean setNeutralBehaviour(NeutralBehaviour mode);

    /**
     * @param rotationsPerMeter The linear movement provided on the output shaft by each rotation of the motor
     * @return If the operation was successful
     */
    boolean setRotationsPerMeter(double rotationsPerMeter);

    /**
     * @param position Forces the encoder to set its current position
     * @return If the operation was successful
     */
    boolean setEncoderPosition(double position);

    boolean setPIDF(PIDFController.Gains gains);
    default boolean setPIDF(final double kP, final double kI, final double kD, final double kFF) {
        return setPIDF(new PIDFController.Gains(kP, kI, kD, kFF));
    }

    boolean setMotionParameters(MotionParameters vars);
    default boolean setMotionParameters(final double maxAcceleration, final double cruiseVelocity, final double allowableError) {
        return setMotionParameters(new MotionParameters(maxAcceleration, cruiseVelocity, allowableError));
    }

    void setNeutral();

    void setDutyCycle(double cycle, double feedforward);
    default void setDutyCycle(final double cycle) {
        setDutyCycle(cycle, 0.0);
    }

    void setVoltage(double voltage, double feedforward);
    default void setVoltage(final double voltage) {
        setVoltage(voltage, 0.0);
    }

    void setVelocityMetersPerSecond(double velocity, double feedforward);
    default void setVelocityMetersPerSecond(final double velocity) {
        setVelocityMetersPerSecond(velocity, 0.0);
    }

    void setVelocityMotorRPM(double rpm, double feedforward);
    default void setVelocityMotorRPM(double rpm) {
        setVelocityMotorRPM(rpm, 0.0);
    }

    default void setVelocityMotorRadiansPerSecond(final double radiansPerSecond) {
        setVelocityMotorRPM(Units.radsPerSec2Rpm(radiansPerSecond));
    }

    void setPositionMeters(double position, double feedforward);
    default void setPositionMeters(final double position) {
        setPositionMeters(position, 0.0);
    }

    void setPositionRotations(double position, double feedforward);
    default void setPositionRotations(final double position) {
        setPositionRotations(position, 0.0);
    }

    double getPositionMeters();
    double getPositionMotorRotations();

    /**
     * @return Current linear velocity in m/s
     */
    double getVelocityLinearMetersPerSecond();
    double getVelocityMotorRPM();

    /**
     * @return radians/s of the motor
     */
    default double getVelocityAngular() {
        return Units.rpm2RadsPerSecond(getVelocityMotorRPM());
    }
}
