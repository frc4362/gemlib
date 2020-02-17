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

	// returns the best chambers to load from
	// in practice, it's the chamber nearest the object
	// which is on the end of the stream of balls
	public List<Chamber> getCandidateChambers(final Location object) {
		final Chamber nearestChamber = getNearestChamber(object);

		if (getFilledChamberCount() == 0) {
			return List.of(nearestChamber);
		}

		return m_chambers.stream()
			 .filter(Chamber::isEmpty)
			 .sorted(Comparator
					 .<Chamber>comparingInt(chamber -> chamber.getTotalDistanceScore(m_chambers))
					 .thenComparingInt(chamber -> chamber.getDistanceScore(nearestChamber)))
			 .collect(Collectors.toList());
	}

	public Optional<Chamber> getOptimalLoadingChamber(final Intake intakeChannel) {
		final List<Chamber> candidates = getCandidateChambers(intakeChannel.getLocation());

		if (candidates.isEmpty()) {
			return Optional.empty();
		} else {
			return Optional.of(candidates.get(candidates.size() - 1));
		}
	}

	// shoots counter-clockwise
	public Optional<Chamber> getOptimalShootingChamber() {
		if (getFilledChamberCount() == 6) {
			return Optional.of(getNearestChamber(Location.SHOOTER));
		}

		final List<Chamber> candidates = getCandidateChambers(Location.SHOOTER);

		if (candidates.size() == 0) {
			return Optional.empty();
		} else if (candidates.size() == 1) {
			return Optional.of(candidates.get(0));
		} else {
			// take two highest scored chambers
			// will always be on either side of the balls
			final List<Chamber> cs = candidates.subList(candidates.size() - 2, candidates.size());
			cs.sort(Comparator.comparing(m_rotatedChambers::indexOf));
			final int indexMin = m_rotatedChambers.indexOf(cs.get(0));
			final int indexMax = m_rotatedChambers.indexOf(cs.get(1));
			final Chamber middleChamber = m_rotatedChambers.get((indexMin + indexMax) / 2);

			return Optional.of(m_rotatedChambers.get(middleChamber.isFull() ? indexMax : indexMin));
		}
	}
}
