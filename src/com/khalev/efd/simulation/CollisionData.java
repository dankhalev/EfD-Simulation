package com.khalev.efd.simulation;

import java.util.ArrayList;
import java.util.Objects;

/**
 * This class represents an input for robot's collision sensor.
 */
public class CollisionData {
    public ArrayList<Double> collisionPoints = new ArrayList<>();
    public Action action;

    public CollisionData(Action action) {
        this.action = action;
    }

    @Override
    public boolean equals(Object o) {
        if (o == this) return true;

        if (!(o instanceof CollisionData)) {
            return false;
        }
        CollisionData inp = (CollisionData) o;
        return action.equals(inp.action) &&
                collisionPoints.equals(inp.collisionPoints);
    }

    @Override
    public int hashCode() {
        return Objects.hash(collisionPoints, action);
    }

    public static CollisionData copy(CollisionData inp) {
        CollisionData collisionData = new CollisionData(new Action(inp.action.velocity, inp.action.angle));
        collisionData.action.degreeOfRealization = inp.action.degreeOfRealization;
        for (int i = 0; i < inp.collisionPoints.size(); i++) {
            collisionData.collisionPoints.add(inp.collisionPoints.get(i).doubleValue());
        }
        return collisionData;
    }
}
