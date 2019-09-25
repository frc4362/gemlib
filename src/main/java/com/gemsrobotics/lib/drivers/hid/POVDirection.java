package com.gemsrobotics.lib.drivers.hid;

public enum POVDirection {
    NONE(-1),
    N(0),
    NE(1),
    E(2),
    SE(3),
    S(4),
    SW(5),
    W(6),
    NW(7);

    private final int m_val;

    POVDirection(final int val) {
        m_val = val;
    }

    public int getValue() {
        return m_val;
    }

    public int getDegrees() {
        return m_val * 45;
    }

    public static POVDirection of(final int val) {
        switch (val) {
        case -1: return NONE;
        case 0: return N;
        case 1: return NE;
        case 2: return E;
        case 3: return SE;
        case 4: return S;
        case 5: return SW;
        case 6: return W;
        case 7: return NW;
        default:
            // this should never happen
            return NONE;
        }
    }

    public static POVDirection ofDegrees(final int degrees) {
        return degrees == -1 ? NONE : of(degrees / 45);
    }

    public int toDegrees() {
        if (m_val == -1) {
            return -1;
        } else {
            return m_val * 45;
        }
    }
}
