package com.gemsrobotics.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.gemsrobotics.frc2018.Ports;
import com.gemsrobotics.lib.drivers.BannerSensor;
import com.gemsrobotics.lib.telemetry.reporting.ReportingEndpoint.Event.Kind;
import com.gemsrobotics.lib.data.CachedValue;
import com.gemsrobotics.lib.structure.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.Objects;

@SuppressWarnings({"unused", "WeakerAccess"})
public final class Carriage extends Subsystem implements Loggable {
	private static Carriage INSTANCE;

	public static Carriage getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new Carriage();
		}

		return INSTANCE;
	}

	private final BannerSensor m_reflectiveSensor;
	private final DoubleSolenoid m_mouth;
	private final TalonSRX m_inner1, m_inner2, m_outer1, m_outer2;
	private final PeriodicIO m_periodicIO;

    public final CachedValue<Boolean> isSensorPresent;

	private MouthState m_mouthState;

	private Carriage() {
	    setName("Carriage");

		m_reflectiveSensor = new BannerSensor(Ports.REFLECTIVE_SENSOR_PORT);

		m_mouth = new DoubleSolenoid(Ports.MOUTH_PORTS[0], Ports.MOUTH_PORTS[1]);

		m_inner1 = new TalonSRX(Ports.INTAKE_INNER_PORT_LEFT);
		m_inner2 = new TalonSRX(Ports.INTAKE_INNER_PORT_RIGHT);
		m_outer1 = new TalonSRX(Ports.INTAKE_OUTER_PORT_LEFT);
		m_outer2 = new TalonSRX(Ports.INTAKE_OUTER_PORT_RIGHT);

		configureTalon(m_inner1, false);
		configureTalon(m_inner2, true);
		configureTalon(m_outer1, false);
		configureTalon(m_outer2, true);

		m_periodicIO = new PeriodicIO();
		isSensorPresent = new CachedValue<>(Boolean.class, 1.0, () -> m_reflectiveSensor.getRawSensor().getVoltage() != 0.0);
	}

    private void configureTalon(final TalonSRX device, final boolean invert) {
		device.setNeutralMode(NeutralMode.Coast);
		device.setInverted(invert);
	}

	public enum MouthState {
		AUTOMATIC,
		FORCE_CLOSE
	}

	public static class PeriodicIO implements Loggable {
        // INPUTS
        @Log(name="Mouth Open? (Boolean)")
		public boolean mouthOpen;
        @Log(name="Mouth Forced Close? (Boolean)")
		public boolean mouthForcedClose;
        @Log(name="Has Cube?")
		public boolean hasCube;

		// OUTPUTS
        @Log(name="Demand (%)")
        public double openLoopDemand;
	}

	@Override
	public synchronized void readPeriodicInputs() {
		m_periodicIO.mouthOpen = m_mouth.get() == DoubleSolenoid.Value.kReverse;
		m_periodicIO.mouthForcedClose = m_mouthState == MouthState.FORCE_CLOSE;
		m_periodicIO.hasCube = m_reflectiveSensor.isBlocked();
	}

	public synchronized boolean isMouthOpen() {
		return m_periodicIO.mouthOpen;
	}

	public synchronized void setWantedMouthState(final MouthState state) {
		if (state != m_mouthState) {
			m_mouthState = state;
		}
	}

	private synchronized void setMouthOpen(final boolean open) {
		if (open != m_periodicIO.mouthOpen) {
			report(Kind.INFO, "Opened: " + open);
			m_mouth.set(open ? Value.kReverse : Value.kForward);
		}
	}

	public synchronized void setOpenLoop(final double speed) {
		m_periodicIO.openLoopDemand = speed;
	}

	@Override
	public synchronized void onCreate(final double timestamp) {
        m_outer2.set(ControlMode.Follower, m_outer1.getDeviceID());
        m_inner2.set(ControlMode.Follower, m_inner1.getDeviceID());
	}

	@Override
	public synchronized void onEnable(final double timestamp) {
        setWantedMouthState(MouthState.AUTOMATIC);
        setOpenLoop(0.0);
	}

	@Override
	public synchronized void onUpdate(final double timestamp) {
        m_outer1.set(ControlMode.PercentOutput, m_periodicIO.openLoopDemand);
        m_inner1.set(ControlMode.PercentOutput, m_periodicIO.mouthOpen ? m_periodicIO.openLoopDemand : 0.0);

        switch (m_mouthState) {
            case AUTOMATIC:
                final var elevator = Elevator.getInstance();
                final var reference = elevator.getReference();
                final var position = elevator.getPosition();

                setMouthOpen((reference <= position && reference < Elevator.Position.CARRY.ticks) || (position < Elevator.Position.CLOSE_THRESHOLD.ticks));

                break;
            case FORCE_CLOSE:
                setMouthOpen(false);
                break;
            default:
                report(Kind.ERROR, "Unexpected state: \"" + m_mouthState + "\".");
                break;
        }
	}

	@Override
	public synchronized void onStop(final double timestamp) {
        setSafeState();
	}

	@Override
	public synchronized void setSafeState() {
		m_inner1.set(ControlMode.Disabled, 0);
		m_outer1.set(ControlMode.Disabled, 0);
	}

	@Override
	public FaultedResponse checkFaulted() {
		if (!isSensorPresent.get() && isSensorPresent.getLastValue().orElse(true)) {
			report(Kind.HARDWARE_FAULT, "Sensor lost");
		} else if (isSensorPresent.get() && !isSensorPresent.getLastValue().orElse(false)) {
			report(Kind.INFO, "Sensor found");
		}

		return FaultedResponse.NONE;
	}

	public synchronized boolean hasCube() {
		return m_periodicIO.hasCube;
	}
}
