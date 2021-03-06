package com.gemsrobotics.frc2020.subsystems;

import java.util.List;

import static java.lang.Math.abs;

public final class Chamber {
	private final int m_index;
	private boolean m_isFull;

	public Chamber(final int id) {
		m_index = id;
		m_isFull = false;
	}

	public boolean isFull() {
		return m_isFull;
	}

	public boolean isEmpty() {
		return !isFull();
	}

	public void setFull(final boolean full) {
		m_isFull = full;
	}

	public int getIndex() {
		return m_index;
	}

	public int getDistance(final Chamber other) {
		int d = other.getIndex() - m_index;

		if (d > 3) {
			d -= 6;
		} else if (d < -3) {
			d += 6;
		}

		return d;
	}

	public int getDistanceScore(final Chamber other) {
		return 4 - abs(getDistance(other));
	}

	public int getTotalDistanceScore(final List<Chamber> chambers) {
		return chambers.stream().filter(Chamber::isFull).mapToInt(this::getDistanceScore).sum();
	}

	@Override
	public boolean equals(final Object other) {
		return other instanceof Chamber && ((Chamber) other).getIndex() == m_index;
	}
}
