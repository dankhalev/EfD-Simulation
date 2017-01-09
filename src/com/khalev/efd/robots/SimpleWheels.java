package com.khalev.efd.robots;

import com.khalev.efd.simulation.Wheels;
import com.khalev.efd.simulation.Action;

public class SimpleWheels implements Wheels {
    private static final double MAX_SPEED = 1.0;
    double rotationAngle = 0.0;
    double speed = 1.0;

    public Action sendCurrentAction() {
        return new Action(this.speed * MAX_SPEED, this.rotationAngle);
    }
}

