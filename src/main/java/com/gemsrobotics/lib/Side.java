package com.gemsrobotics.lib;

public enum Side {
    LEFT(1), RIGHT(-1);

    public final int rotationMultiplier;

    Side(final int rotationMultiplier) {
        this.rotationMultiplier = rotationMultiplier;
    }

    public boolean isLeft() {
        return this == LEFT;
    }

    public boolean isRight() {
        return !isLeft();
    }
    // comment somewhere or something
}
