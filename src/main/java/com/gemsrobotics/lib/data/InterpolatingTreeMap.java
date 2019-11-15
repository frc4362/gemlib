package com.gemsrobotics.lib.data;

import com.gemsrobotics.lib.math.interpolation.Interpolatable;
import com.gemsrobotics.lib.math.interpolation.InverseInterpolatable;

import java.util.Map;
import java.util.TreeMap;

/**
 * Interpolating Tree Maps are used to get values at points that are not defined by making a guess from points that are
 * defined. This uses linear interpolation.
 * 
 * @param <K>
 *            The type of the key (must implement InverseInterpolable)
 * @param <V>
 *            The type of the value (must implement Interpolable)
 */
public class InterpolatingTreeMap<
                 K extends InverseInterpolatable<K> & Comparable<K>,
                 V extends Interpolatable<V>>
        extends TreeMap<K, V> {
    private static final long serialVersionUID = 8347275262778054124L;

    private final int m_maximumSize;

    public InterpolatingTreeMap(int maximumSize) {
        this.m_maximumSize = maximumSize;
    }

    /**
     * Inserts a key value pair, and trims the tree if a max size is specified
     * 
     * @param key
     *            Key for inserted data
     * @param value
     *            Value for inserted data
     * @return the value
     */
    @Override
    public V put(final K key, final V value) {
        if (m_maximumSize > 0 && m_maximumSize <= size()) {
            // "Prune" the tree if it is oversize
            final K first = firstKey();
            remove(first);
        }

        super.put(key, value);

        return value;
    }

    @Override
    public void putAll(Map<? extends K, ? extends V> map) {
        map.forEach(this::put);
    }

    /**
     *
     * @param key Lookup or calculate a value
     * @return V or null; V if it is calculable or exists, null if it is at a bound and cannot average
     */
    public V getInterpolated(final K key) {
        final V foundValue = get(key);

        if (foundValue == null) {
            // Get closest keys for interpolation
            K topBound = ceilingKey(key);
            K bottomBound = floorKey(key);

            // If attempting interpolation at bounds of the tree, return the nearest data point
            if (topBound == null && bottomBound == null) {
                return null;
            } else if (topBound == null) {
                return get(bottomBound);
            } else if (bottomBound == null) {
                return get(topBound);
            }

            // Get surrounding values for interpolation
            final V topElem = get(topBound);
            final V bottomElem = get(bottomBound);
            return bottomElem.interpolate(topElem, bottomBound.inverseInterpolate(topBound, key));
        } else {
            return foundValue;
        }
    }

    public int getMaximumSize() {
        return m_maximumSize;
    }
}
