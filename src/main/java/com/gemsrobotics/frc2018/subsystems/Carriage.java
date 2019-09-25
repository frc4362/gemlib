package com.gemsrobotics.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.gemsrobotics.frc2018.Ports;
import com.gemsrobotics.lib.telemetry.reporting.Reporter.Event.Kind;
import com.gemsrobotics.lib.property.CachedValue;
import com.gemsrobotics.lib.structure.Subsystem;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.Map;
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

	private static final double
			VOLTAGE_DETECTION_THRESHOLD = 0.1;

	public final CachedValue<Boolean> isSensorPresent;

	private final AnalogInput m_reflectiveSensor;
	private final DoubleSolenoid m_mouth;
	private final TalonSRX m_inner1, m_inner2, m_outer1, m_outer2;

	private final PeriodicIO m_periodicIO;

	private MouthState m_mouthState;

	private Carriage() {
	    setName("Carriage");

		m_reflectiveSensor = new AnalogInput(Ports.REFLECTIVE_SENSOR_PORT);

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

		isSensorPresent = new CachedValue<>(Boolean.class, 1.0, () -> m_reflectiveSensor.getVoltage() != 0.0);
	}

    @Override
    protected void initDefaultCommand() { }

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
        @Log(name="Sensor Value (V)")
		public double sensorAverageVoltage;

		// OUTPUTS
        @Log(name="Demand (%)")
        public double openLoopDemand;
	}

	@Override
	public void readPeriodicInputs() {
		m_periodicIO.mouthOpen = m_mouth.get() == DoubleSolenoid.Value.kReverse;
		m_periodicIO.mouthForcedClose = m_mouthState == MouthState.FORCE_CLOSE;
		m_periodicIO.sensorAverageVoltage = m_reflectiveSensor.getAverageVoltage();
	}

	public boolean isMouthOpen() {
		return m_periodicIO.mouthOpen;
	}

	public void setWantedMouthState(final MouthState state) {
		if (state != m_mouthState) {
			m_mouthState = state;
		}
	}

	private void setMouthOpen(final boolean open) {
		if (open != m_periodicIO.mouthOpen) {
			report(Kind.INFO, "Opened: " + open);
			m_mouth.set(open ? Value.kReverse : Value.kForward);
		}
	}

	public void setOpenLoop(final double speed) {
		m_periodicIO.openLoopDemand = speed;
	}

	@Override
	public void onCreate(final double timestamp) {
		synchronized (this) {
			setWantedMouthState(MouthState.AUTOMATIC);
			setOpenLoop(0.0);
		}
	}

	@Override
	public void onEnable(final double timestamp) {
		synchronized (this) {
            m_outer2.set(ControlMode.Follower, m_outer1.getDeviceID());
            m_inner2.set(ControlMode.Follower, m_inner1.getDeviceID());
        }
	}

	@Override
	public void onUpdate(final double timestamp) {
		synchronized (this) {
			switch (m_mouthState) {
				case AUTOMATIC:
					final var elevator = Elevator.getInstance();
					final var reference = elevator.getReference();
					final var position = elevator.getPosition();

					if ((reference <= position && reference < Elevator.Position.CARRY.positionTicks)
						 || (position < Elevator.Position.CLOSE_THRESHOLD.positionTicks)
					) {
						setMouthOpen(true);
					} else {
						setMouthOpen(false);
					}

					break;
				case FORCE_CLOSE:
					setMouthOpen(false);
					break;
				default:
					report(Kind.ERROR, "Unexpected state: \"" + m_mouthState + "\".");
					break;
			}

            m_outer1.set(ControlMode.PercentOutput, m_periodicIO.openLoopDemand);
            m_inner1.set(ControlMode.PercentOutput, m_periodicIO.mouthOpen ? m_periodicIO.openLoopDemand : 0.0);
        }
	}

	@Override
	public void onStop(final double timestamp) {
		synchronized (this) {
			setSafeState();
		}
	}

	@Override
	public void setSafeState() {
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

	public boolean hasCube() {
		return m_periodicIO.sensorAverageVoltage > VOLTAGE_DETECTION_THRESHOLD;
	}
}
