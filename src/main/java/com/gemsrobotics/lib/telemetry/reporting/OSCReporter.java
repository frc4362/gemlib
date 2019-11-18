package com.gemsrobotics.lib.telemetry.reporting;

import com.gemsrobotics.lib.telemetry.osc.OSCClientPool;
import com.gemsrobotics.lib.telemetry.osc.OSCConstants;
import com.illposed.osc.*;

import java.util.Arrays;
import java.util.Objects;
import java.util.stream.Collectors;

public final class OSCReporter extends ReportingEndpoint {
	private static OSCReporter INSTANCE;

	public static OSCReporter getInstance() {
		if (Objects.isNull(INSTANCE)) {
			INSTANCE = new OSCReporter();
		}

		return INSTANCE;
	}

	private final OSCClientPool m_clients;
    private OSCPortIn m_oscInput;

	private OSCReporter() {
        m_clients = new OSCClientPool();

        try {
            m_oscInput = new OSCPortIn(OSCConstants.OSC_PORT);

            final OSCListener listener = (time, message) -> {
                try {
                    m_clients.add(message.getIPAddress());
                } catch (final Exception ignored) {

                }
            };

            m_oscInput.addListener("/Reporting", listener);
            m_oscInput.startListening();
        } catch (final Exception exception) {
            if (!Objects.isNull(m_oscInput)) {
                try {
                    m_oscInput.stopListening();
                    m_oscInput.close();
                } catch (final Exception ignored) {

                }
            }
        }
	}

	@Override
	protected void doWrite(final Event[] events) {
	    if (!m_clients.hasClients()) {
	        return;
        }

	    final OSCBoundListMessage message = new OSCBoundListMessage("/ReportingOutput", Arrays.stream(events)
                .map(ConsoleReporter::event2String)
                .collect(Collectors.toList()));

        m_clients.dropExpiredClients();
        m_clients.forEach(client -> {
            boolean doSend = true;

            if (!Objects.isNull(client.getAddress())) {
                if (Objects.isNull(client.getOutputPort())) {
                    try {
                        client.setOutputPort(new OSCPortOut(client.getAddress(), OSCConstants.OSC_PORT));
                    } catch (final Exception exception) {
                        doSend = false;
                    }
                }
            }

            if (doSend) {
                try {
                    client.getOutputPort().send(message);
                } catch (final Exception ignored) {

                }
            }
        });
	}
}
