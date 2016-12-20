package com.khalev.efd.robots;

import com.khalev.efd.simulation.Action;
import com.khalev.efd.simulation.SpatialInput;

import java.util.ArrayList;
import java.util.Objects;

public class SimpleCollisionSensor implements com.khalev.efd.simulation.CollisionSensor {
    ArrayList<Double> collisionPoints = new ArrayList<>();
    Action previousAction = new Action(0,0);
    SpatialInput input = new SpatialInput(new Action(0,0));

    public void receiveSpatialInput(SpatialInput input) {
        this.collisionPoints = input.collisionPoints;
        this.previousAction = input.action;
        this.input = input;
    }

    public boolean inputReceived(SpatialInput input) {

        if (this.collisionPoints.size() != input.collisionPoints.size()) {
            return false;
        } else for (int i = 0; i < this.collisionPoints.size(); i++) {
            if (!Objects.equals(this.collisionPoints.get(i), input.collisionPoints.get(i))) {
                return false;
            }
        }
        if (!this.previousAction.isIdenticalTo(input.action)) {
            return false;
        }

        if (!this.input.isIdenticalTo(input)) {
            return false;
        }

        return true;
    }

}
