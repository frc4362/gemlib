package com.gemsrobotics.frc2019.util;

import java.util.LinkedList;

public class LimitedQueue<E> extends LinkedList<E> {
	private int m_limit;

	public LimitedQueue(int limit) {
		m_limit = limit;
	}

	@Override
	public boolean add(E o) {
		boolean added = super.add(o);

		while (added && size() > m_limit) {
			super.remove();
		}

		return added;
	}
}