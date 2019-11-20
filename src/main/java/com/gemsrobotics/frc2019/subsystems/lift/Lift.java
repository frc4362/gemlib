package com.gemsrobotics.frc2019.subsystems.lift;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;

import static java.lang.Double.max;
import static java.lang.Math.abs;
import static java.lang.Math.min;

@SuppressWarnings({"unused", "WeakerAccess"})
public class Lift implements Sendable {
	private static final double STOP_THRESHOLD = 0.2;

	public enum Position {
		PANEL_3(0.704),
		PANEL_2(0.352),
		PANEL_1(0.0),
		CARGO_3(0.968),
		CARGO_2(0.613),
		CARGO_1(0.264),
		CARGO_SHIP(0.4454),
		STAGE1_RETRACT_DISTANCE(0.05),
		BOTTOM(0.0);

		private static double TOP_INCHES = 79.5;

		private static double fromInches(final double inches) {
			return inches / TOP_INCHES;
		}

		public final double percent;

		Position(final double pos) {
			percent = pos;
		}
	}

	private static double constrain(final double bot, final double val, final double top) {
		return min(max(val, bot), top);
	}

	private final CANSparkMax m_motorMaster, m_motorSlave;
	private final LiftConfig m_liftConfig;

	private String m_name, m_subsystem;
	private Position m_setPosition;
	private double m_setpoint;

	public Lift(final LiftConfig liftConfig) {
		setName("Lift");
		setSubsystem("Lift");

		m_liftConfig = liftConfig;

		m_motorMaster = new CANSparkMax(m_liftConfig.portMaster, MotorType.kBrushless);
		m_motorSlave = new CANSparkMax(m_liftConfig.portSlave, MotorType.kBrushless);

		Arrays.asList(m_motorMaster, m_motorSlave).forEach(motor -> {
			m_liftConfig.pidVars.configure(motor.getPIDController());
			motor.getPIDController().setOutputRange(-1.0, 1.0);
		});

		m_motorMaster.setInverted(true);
		m_motorSlave.setInverted(true);
		m_motorSlave.follow(m_motorMaster, true);

		m_setPosition = Position.BOTTOM;
		m_setpoint = 0.0;
	}

	public void setIdleMode(final CANSparkMax.IdleMode idleMode) {
		m_motorMaster.setIdleMode(idleMode);
		m_motorSlave.setIdleMode(idleMode);
	}

	private double toRotations(final double percent) {
		return percent * m_liftConfig.totalRotations() + m_liftConfig.rotationsBottom;
	}

	private void setRotations(final double rotations) {
		final var setpoint = constrain(
				Position.BOTTOM.percent * m_liftConfig.totalRotations(),
				rotations,
				Position.CARGO_3.percent * m_liftConfig.totalRotations()
		);

		m_setpoint = setpoint;
		m_motorMaster.getPIDController().setReference(setpoint, ControlType.kPosition);
	}

	public void setPosition(final Position pos) {
		setRotations(toRotations(pos.percent));
	}

	private void setPercent(final double val) {
		setRotations(toRotations(val));
	}

	public void set(final double val) {
		m_motorMaster.set(val);
	}

	public double getPosition() {
		return m_motorMaster.getEncoder().getPosition();
	}

	public void adjustPosition(final double percent) {
		setRotations(m_setpoint + toRotations(percent));
	}

	public void pause() {
		m_motorMaster.getPIDController().setReference(0.0, ControlType.kVoltage);
	}

	public boolean isAtSetpoint() {
		return abs(getPosition() - m_setpoint) < STOP_THRESHOLD;
	}

	public boolean isBlockingCamera() {
		final var pos = getPosition();
		return pos < m_liftConfig.cameraBlockedTop && pos > m_liftConfig.cameraBlockedBottom;
	}

	public double heightRotations(final Position position) {
		return position.percent * m_liftConfig.totalRotations();
	}

	@Override
	public String getName() {
		return m_name;
	}

	@Override
	public void setName(final String name) {
		m_name = name;
	}

	@Override
	public String getSubsystem() {
		return m_subsystem;
	}

	@Override
	public void setSubsystem(final String subsystem) {
		m_subsystem = subsystem;
	}

	@Override
	public void initSendable(final SendableBuilder builder) {
		builder.setSmartDashboardType("Lift");
		builder.getEntry("Ports").setString(String.format("[%d, %d]",
				m_motorMaster.getDeviceId(), m_motorSlave.getDeviceId()));

		final var positionEntry = builder.getEntry("Position");
		final var setPositionEntry = builder.getEntry("Set Position (maybe)");

		builder.setUpdateTable(() -> {
			positionEntry.setString(Double.toString(m_motorMaster.getEncoder().getPosition()));
			setPositionEntry.setString(Double.toString(m_motorMaster.get()));
		});
	}

	public LiftTuner getTunerCommand() {
		return new LiftTuner(this);
	}

	private static class LiftTuner extends Command {
		private double kP, kI, kD, kF, oldSetpoint;

		private final Lift m_lift;
		private final Preferences prefs = Preferences.getInstance();

		public LiftTuner(final Lift lift) {
			m_lift = lift;
		}

		@Override
		public void execute() {
			final double p = prefs.getDouble("p", 0);
			final double i = prefs.getDouble("i", 0);
			final double d = prefs.getDouble("d", 0);
			final double f = prefs.getDouble("f", 0);
			final double setpoint = prefs.getDouble("setpoint", 0);

			SmartDashboard.putNumber("Lift Setpoint Rots",
					setpoint * m_lift.m_liftConfig.totalRotations());

			if (p != kP) {
				kP = p;
				m_lift.m_motorMaster.getPIDController().setP(kP);
			}

			if (i != kI) {
				kI = i;
				m_lift.m_motorMaster.getPIDController().setI(kI);
			}

			if (d != kD) {
				kD = d;
				m_lift.m_motorMaster.getPIDController().setD(kD);
			}

			if (f != kF) {
				kF = f;
				m_lift.m_motorMaster.getPIDController().setFF(kF);
			}

			if (setpoint != oldSetpoint) {
				oldSetpoint = setpoint;
				m_lift.setPercent(setpoint);
			}
		}

		@Override
		public boolean isFinished() {
			return false;
		}
	}

	private static class LiftLogger extends Command {
		private final Lift m_lift;

		public LiftLogger(final Lift lift) {
			m_lift = lift;
		}

		@Override
		public void execute() {
			SmartDashboard.putNumber("Lift Position", m_lift.getPosition());
		}

		@Override
		public boolean isFinished() {
			return false;
		}
	}

	public Command makeLogger() {
		return new LiftLogger(this);
	}

	public Position getSetPosition() {
		return m_setPosition;
	}
}
