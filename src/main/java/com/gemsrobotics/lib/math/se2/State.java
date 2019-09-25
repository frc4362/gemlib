package com.gemsrobotics.lib.math.se2;

import com.gemsrobotics.lib.math.interpolation.Interpolable;

public interface State<S> extends Interpolable<S> {
    double distance(S other);

    boolean equals(Object other);

    String toString();
}
