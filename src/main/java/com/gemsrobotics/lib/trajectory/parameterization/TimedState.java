package com.gemsrobotics.lib.trajectory.parameterization;

import com.gemsrobotics.lib.utils.FastDoubleToString;
import com.gemsrobotics.lib.math.se2.State;
import com.google.gson.annotations.SerializedName;

import java.util.Objects;

import static com.gemsrobotics.lib.utils.MathUtils.epsilonEquals;
import static com.gemsrobotics.lib.utils.MathUtils.lerp;

@SuppressWarnings("WeakerAccess")
public class TimedState<S extends State<S>> implements State<TimedState<S>> {
    @SerializedName("state")
    protected final S m_state;
    @SerializedName("time")
    protected double m_t; // Time we achieve this state.
    @SerializedName("velocity")
    protected double m_velocity; // ds/dt
    @SerializedName("acceleration")
    protected double m_acceleration; // d^2s/dt^2

    public TimedState(final S state) {
        m_state = state;
    }

    public TimedState(
            final S state,
            final double t,
            final double velocity,
            final double acceleration
    ) {
        m_state = state;
        m_t = t;
        m_velocity = velocity;
        m_acceleration = acceleration;
    }

    public S getState() {
        return m_state;
    }

    public void setT(final double t) {
        m_t = t;
    }

    public double t() {
        return m_t;
    }

    public void setVelocity(final double velocity) {
        m_velocity = velocity;
    }

    public double getVelocity() {
        return m_velocity;
    }

    public void setAcceleration(final double acceleration) {
        m_acceleration = acceleration;
    }

    public double getAcceleration() {
        return m_acceleration;
    }

    @Override
    public String toString() {
        return "[" + getState().toString() +
               ", t: " + FastDoubleToString.format(t()) +
               ", v: " + FastDoubleToString.format(getVelocity()) +
               ", a: " + FastDoubleToString.format(getAcceleration()) + "]";
    }

    @Override
    public TimedState<S> interpolate(final TimedState<S> other, final double x) {
        final double newT = lerp(t(), other.t(), x);
        final double deltaT = newT - t();

        if (deltaT < 0.0) {
            return other.interpolate(this, 1.0 - x);
        }

        final var reversing = getVelocity() < 0.0 || (epsilonEquals(getVelocity(), 0.0) && getAcceleration() < 0.0);
        final double newV = getVelocity() + getAcceleration() * deltaT;
        final double newS = (reversing ? -1.0 : 1.0) * (getVelocity() * deltaT + .5 * getAcceleration() * deltaT * deltaT);

        return new TimedState<>(
                getState().interpolate(other.getState(),
                newS / getState().distance(other.getState())),
                newT,
                newV,
                getAcceleration());
    }

    @Override
    public double distance(final TimedState<S> other) {
        return getState().distance(other.getState());
    }

    @Override
    public boolean equals(final Object other) {
        if (Objects.isNull(other) || !(other instanceof TimedState<?>)) {
            return false;
        }

        final TimedState<?> ts = (TimedState<?>) other;
        return getState().equals(ts.getState()) && epsilonEquals(t(), ts.t());
    }
}
