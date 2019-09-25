package com.gemsrobotics.lib.data;

import java.util.LinkedList;

public class LimitedQueue<E> extends LinkedList<E> {
	private int m_limit;

	public LimitedQueue(final int capacity) {
		m_limit = capacity;
	}

	@Override
	public boolean add(final E o) {
		boolean added = super.add(o);

		while (added && size() > m_limit) {
			super.remove();
		}

		return added;
	}

	public boolean hasSpaceRemaining() {
	    return size() < m_limit;
    }
}
