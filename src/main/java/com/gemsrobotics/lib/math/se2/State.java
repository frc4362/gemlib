package com.gemsrobotics.lib.math.se2;

import com.gemsrobotics.lib.math.interpolation.Interpolatable;

public interface State<S> extends Interpolatable<S> {
    double distance(S other);

    boolean equals(Object other);

    String toString();
}
