package com.gemsrobotics.lib.math.se2;

public interface ICurvature<S> extends State<S> {
    double getCurvature();
    double getDCurvatureDs();
}
