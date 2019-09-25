package com.gemsrobotics.lib.telemetry.osc;

import com.gemsrobotics.lib.telemetry.Pod;
import com.gemsrobotics.lib.timing.ElapsedTimer;
import com.illposed.osc.OSCPortOut;

import java.io.IOException;
import java.net.InetAddress;

public final class OSCClient {
	private final InetAddress m_address;
	private final ElapsedTimer m_timer;
	private transient OSCPortOut m_outputPort;

	public OSCClient(final InetAddress address) {
		m_address = address;
		m_timer = new ElapsedTimer(4.0);

		try {
		    m_outputPort = new OSCPortOut(address, OSCConstants.OSC_PORT);
        } catch (final IOException e) {
            Pod.catchThrowable(this, e);
            m_outputPort = null;
        }
    }

    public InetAddress getAddress() {
	    return m_address;
    }

    public OSCPortOut getOutputPort() {
	    return m_outputPort;
    }

    public void setOutputPort(final OSCPortOut port) {
	    m_outputPort = port;
    }

    public synchronized boolean isExpired() {
	    return m_timer.hasElapsed();
    }

    public synchronized void heartbeat() {
	    m_timer.reset();
    }

    @Override
    public boolean equals(final Object other) {
	    if (other instanceof OSCClient) {
	        return m_address.equals(((OSCClient) other).m_address);
        } else if (other instanceof InetAddress) {
	        return m_address.equals(((InetAddress) other));
        } else {
	        return false;
        }
    }

    @Override
    public int hashCode() {
	    return m_address.hashCode();
    }
}
