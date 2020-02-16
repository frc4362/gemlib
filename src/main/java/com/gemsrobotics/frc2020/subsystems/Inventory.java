package com.gemsrobotics.frc2020.subsystems;

import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import static java.lang.Math.abs;

public final class Inventory {
	public enum Location {
		LEFT_INTAKE(2),
		CENTER_INTAKE(1),
		RIGHT_INTAKE(0),
		SHOOTER(4);

		public final int index;

		Location(final int idx) {
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
		final int chamberRotations = (int) Math.round((rotations % 1.0) * 6.0);
		final List<Chamber> copy = new ArrayList<>(m_chambers);
		Collections.rotate(copy, chamberRotations);
		m_rotatedChambers = copy;
	}

	public long getFilledChamberCount() {
		return m_chambers.stream().filter(Chamber::isFull).count();
	}

	public Chamber getNearestChamber(final Location object) {
		return m_rotatedChambers.get(object.index);
	}

	public List<Chamber> getCandidateChambers(final Location object) {
		if (getFilledChamberCount() == 0) {
			return List.of(getNearestChamber(object));
		}

		final Chamber nearestChamber = getNearestChamber(object);
		final Comparator<Chamber> utility = Comparator
			 .<Chamber>comparingInt(chamber -> chamber.getTotalDistanceScore(m_chambers))
			 .thenComparingInt(chamber -> chamber.getDistanceScore(nearestChamber));

		return new ArrayList<>(m_chambers)
			 .stream()
			 .filter(Chamber::isEmpty)
			 .sorted(utility)
			 .collect(Collectors.toList());
	}

	public Optional<Chamber> getOptimalLoadingChamber(final Location object) {
		final List<Chamber> candidates = getCandidateChambers(object);
		// highest-scoring element will be last
		if (candidates.size() > 0) {
			return Optional.of(candidates.get(candidates.size() - 1));
		} else {
			return Optional.empty();
		}
	}

	// shoots counter-clockwise
	public Optional<Chamber> getOptimalShootingChamber() {
		final List<Chamber> candidates = getCandidateChambers(Location.SHOOTER);

		if (candidates.size() == 0) {
			return Optional.empty();
		} else if (candidates.size() == 1) {
			return Optional.of(candidates.get(0));
		} else {
			// take two highest scored chambers
			final List<Chamber> cs = candidates.subList(candidates.size() - 2, candidates.size());
			cs.sort(Comparator.comparing(m_rotatedChambers::indexOf));
			final int indexMin = m_rotatedChambers.indexOf(cs.get(0));
			final int indexMax = m_rotatedChambers.indexOf(cs.get(1));
			final Chamber middleChamber = m_rotatedChambers.get((indexMin + indexMax) / 2);

			if (middleChamber.isFull()) {
				return Optional.of(m_rotatedChambers.get(indexMax));
			} else {
				return Optional.of(m_rotatedChambers.get(indexMin));
			}
		}
	}
}
