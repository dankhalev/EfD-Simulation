package com.khalev.efd.simulation;

public class Line {

    double start;
    double end;
    boolean isVertical;
    int horizon;

    public Line(double start, double end, int horizon, boolean isVertical) {
        this.start = start;
        this.end = end;
        this.isVertical = isVertical;
        this.horizon = horizon;
    }
}
