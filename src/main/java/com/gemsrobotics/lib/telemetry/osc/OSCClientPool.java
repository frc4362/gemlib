package com.gemsrobotics.lib.telemetry.osc;

import java.net.InetAddress;
import java.util.Objects;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;
import java.util.function.Consumer;

public class OSCClientPool {
    private final ConcurrentMap<OSCClient, OSCClient> m_clients;

    public OSCClientPool() {
        m_clients = new ConcurrentHashMap<>();
    }

    public synchronized OSCClient add(final InetAddress address) {
        if (!Objects.isNull(address)) {
            try {
                if (!m_clients.containsKey(address)) {
                    return add(new OSCClient(address));
                } else {
                    final var client = m_clients.get(address);
                    client.heartbeat();
                    return client;
                }
            } catch (final Throwable ignored) {

            }
        }

        return null;
    }

    public synchronized OSCClient add(final OSCClient client) {
        if (!Objects.isNull(client)) {
            try {
                m_clients.putIfAbsent(client, client);

                final var c = m_clients.get(client);
                c.heartbeat();
                return c;
            } catch (final Throwable ignored) {

            }
        }

        return null;
    }

    public synchronized void dropExpiredClients() {
        m_clients.entrySet().removeIf(client -> client.getKey().isExpired());
    }

    public synchronized void forEach(final Consumer<? super OSCClient> action) {
        m_clients.forEach((k, v) -> action.accept(k));
    }

    public boolean hasClients() {
        return m_clients.size() > 0;
    }
}
