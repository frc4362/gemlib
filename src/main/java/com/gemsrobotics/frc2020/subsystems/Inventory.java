package com.gemsrobotics.frc2020.subsystems;

import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public final class Inventory {
	public enum Surroundings {
		LEFT_INTAKE(2),
		CENTER_INTAKE(1),
		RIGHT_INTAKE(0),
		SHOOTER(4);

		public final int index;

		Surroundings(final int idx) {
			index = idx;
		}
	}

	private final List<Chamber> m_chambers;
	private List<Chamber> m_rotatedChambers;

	public Inventory() {
		m_chambers = IntStream.range(0, 6).mapToObj(Chamber::new).collect(Collectors.toList());
		setRotations(0);
	}

	public void setRotations(final double rotations) {
		final List<Chamber> copy = new ArrayList<>(m_chambers);
		final int chamberRotations = (int) Math.round((rotations % 1.0) * 6.0);
		Collections.rotate(copy, chamberRotations);
		m_rotatedChambers = copy;
	}

	public long getFilledChamberCount() {
		return m_chambers.stream().filter(Chamber::isFull).count();
	}

	public long getEmptyChamberCount() {
		return 6 - getFilledChamberCount();
	}

	public Chamber getNearestChamber(final Surroundings object) {
		return m_rotatedChambers.get(object.index);
	}

	public Optional<Chamber> getOptimalFillableChamber(final Surroundings object) {
		if (getFilledChamberCount() == 0) {
			return Optional.of(getNearestChamber(object));
		} else if (getEmptyChamberCount() == 0) {
			return Optional.empty();
		}

		final List<Chamber> candidates = new ArrayList<>(m_chambers)
			   .stream()
			   .filter(Chamber::isEmpty)
			   .sorted(Comparator.comparingInt(chamber -> chamber.getProximityScore(m_chambers)))
			   .collect(Collectors.toList());

		return Optional.of(candidates.get(candidates.size() - 1));
	}
}
