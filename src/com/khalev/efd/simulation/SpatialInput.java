package com.khalev.efd.simulation;

import java.util.ArrayList;
import java.util.Objects;

/**
 * This class represents an input for robot's collision sensor.
 */
public class SpatialInput {
    public ArrayList<Double> collisionPoints = new ArrayList<>();
    public Action action;

    public SpatialInput(Action action) {
        this.action = action;
    }

    public boolean isIdenticalTo(SpatialInput inp) {
        if (!action.isIdenticalTo(inp.action)) {
            return false;
        }
        if (this.collisionPoints.size() != inp.collisionPoints.size()) {
            return false;
        } else for (int i = 0; i < this.collisionPoints.size(); i++) {
            if (!Objects.equals(this.collisionPoints.get(i), inp.collisionPoints.get(i))) {
                return false;
            }
        }
        return true;
    }
}
