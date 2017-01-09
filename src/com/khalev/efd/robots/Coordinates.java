package com.khalev.efd.robots;

import com.khalev.efd.simulation.Action;

import java.util.Objects;

public class Coordinates {
    double x, y, angle;

    public Coordinates(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

    @Override
    public boolean equals(Object o) {
        if (o == this) return true;

        if (!(o instanceof Coordinates)) {
            return false;
        }
        Coordinates c = (Coordinates) o;
        return x == c.x && y == c.y && angle == c.angle;
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y, angle);
    }

}
