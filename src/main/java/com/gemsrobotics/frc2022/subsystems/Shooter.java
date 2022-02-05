package com.gemsrobotics.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.gemsrobotics.lib.controls.MotorFeedforward;
import com.gemsrobotics.lib.controls.PIDFController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorController;
import com.gemsrobotics.lib.drivers.motorcontrol.MotorControllerFactory;
import com.gemsrobotics.lib.structure.Subsystem;
import com.gemsrobotics.lib.utils.Units;
import edu.wpi.first.wpilibj.MedianFilter;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.Objects;

import static com.gemsrobotics.lib.utils.MathUtils.allCloseTo;
import static com.gemsrobotics.lib.utils.MathUtils.epsilonEquals;

public final class Shooter extends Subsystem implements Loggable {
    private static final int
            SHOOTER_MASTER_PORT = 6,
            SHOOTER_SLAVE_PORT = 7;
    private static final double SHOOTER_WHEEL_RADIUS = Units.inches2Meters(3.8) / 2.0;
    private static final PIDFController.Gains SHOOTER_GAINS = new PIDFController.Gains(0.0, 0.0, 0.0, 0.0);
    private static final MotorFeedforward SHOOTER_FEEDFORWARD = new MotorFeedforward(0.0, 0.0, 0.0);
    private static final int MEDIAN_SAMPLES = 15;
    private static final double ALLOWABLE_RPM_ERROR = 200.0;

    private static Shooter INSTANCE;

    public static Shooter getInstance() {
        if (Objects.isNull(INSTANCE)) {
            INSTANCE = new Shooter();
        }

        return INSTANCE;
    }

    private final MotorController<TalonFX>
            m_shooterMaster,
            m_shooterSlave;
    private final MedianFilter m_shooterFilter;
    private final PeriodicIO m_periodicIO;

    private Shooter() {
        m_shooterMaster = MotorControllerFactory.createDefaultTalonFX(SHOOTER_MASTER_PORT);
        m_shooterMaster.setGearingParameters(1.0, SHOOTER_WHEEL_RADIUS, 2048);
        m_shooterMaster.setNeutralBehaviour(MotorController.NeutralBehaviour.COAST);
        m_shooterMaster.setInvertedOutput(false);
        m_shooterMaster.setSelectedProfile(0);
        m_shooterMaster.setPIDF(SHOOTER_GAINS);

        m_shooterSlave = MotorControllerFactory.createDefaultTalonFX(SHOOTER_SLAVE_PORT);
        m_shooterSlave.setGearingParameters(1.0, SHOOTER_WHEEL_RADIUS, 2048);
        m_shooterSlave.setNeutralBehaviour(MotorController.NeutralBehaviour.COAST);
        m_shooterSlave.follow(m_shooterMaster, true);

        m_shooterFilter = new MedianFilter(MEDIAN_SAMPLES);

        m_periodicIO = new PeriodicIO();
    }

    private static class PeriodicIO implements Loggable {
        @Log(name="Shooter Velocity (RPM)")
        public double shooterMeasuredRPM = 0.0;
        public double shooterFilteredRPM = 0.0;
        @Log(name="Shooter Reference (RPM)")
        public double shooterReferenceRPM = 0.0;
        @Log(name="Shooter Current Draw (amps)")
        public double shooterCurrent = 0.0;
        @Log(name="Enabled?")
        public boolean enabled = false;
        public boolean atReference = false;
    }

    @Override
    protected synchronized void readPeriodicInputs(final double timestamp) {
        m_periodicIO.shooterMeasuredRPM = m_shooterMaster.getVelocityAngularRPM();
        m_periodicIO.shooterCurrent = m_shooterMaster.getDrawnCurrentAmps() + m_shooterSlave.getDrawnCurrentAmps();
        m_periodicIO.shooterFilteredRPM = m_shooterFilter.calculate(m_periodicIO.shooterMeasuredRPM);
    }

    public synchronized void setRPM(final double shooterRPM) {
        m_periodicIO.enabled = shooterRPM != 0.0;
        m_periodicIO.shooterReferenceRPM = shooterRPM;
    }

    public synchronized void setDisabled() {
        setRPM(0);
    }

    @Override
    protected synchronized void onStart(final double timestamp) {
        setDisabled();
    }

    @Override
    protected synchronized void onUpdate(final double timestamp) {
        if (m_periodicIO.enabled) {
            final double accelerationSetpoint = (m_periodicIO.shooterReferenceRPM - m_periodicIO.shooterFilteredRPM) / dt();
            final double shooterFeedforward = SHOOTER_FEEDFORWARD.calculateVolts(
                    Units.rpm2RadsPerSecond(m_periodicIO.shooterReferenceRPM),
                    Units.rpm2RadsPerSecond(accelerationSetpoint)) / 12.0;
            m_shooterMaster.setVelocityRPM(m_periodicIO.shooterReferenceRPM, shooterFeedforward);
        } else {
            m_shooterMaster.setNeutral();
        }
    }

    @Override
    protected synchronized void onStop(final double timestamp) {
        setDisabled();
    }

    @Override
    public void setSafeState() {
        m_shooterMaster.setNeutral();
    }

    public synchronized boolean atReference() {
        return epsilonEquals(m_periodicIO.shooterReferenceRPM, m_periodicIO.shooterFilteredRPM, ALLOWABLE_RPM_ERROR);
    }

    public synchronized boolean isNeutral() {
        return m_periodicIO.shooterReferenceRPM == 0.0;
    }
}
