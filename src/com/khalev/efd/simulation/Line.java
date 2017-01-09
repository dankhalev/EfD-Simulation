package com.khalev.efd.simulation;

class Line {

    double start;
    double end;
    boolean isVertical;
    int horizon;

    Line(double start, double end, int horizon, boolean isVertical) {
        this.start = start;
        this.end = end;
        this.isVertical = isVertical;
        this.horizon = horizon;
    }
}
