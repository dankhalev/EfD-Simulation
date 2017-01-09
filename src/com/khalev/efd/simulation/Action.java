package com.khalev.efd.simulation;

import java.util.Objects;

/**
 * This class represents an action that a robot can perform in a cycle of simulation.
 */
public class Action {

    private static final double MAX_SPEED = 1.0;


    public Type type;

    public double velocity;
    public double angle;
    public double degreeOfRealization;

    public Action(double velocity, double rotation) {
        if (velocity != 0) {
            type = Type.MOVE;
            if (velocity > MAX_SPEED) {
                this.velocity = MAX_SPEED;
            } else {
                this.velocity = velocity;
            }
            angle = 0;
        } else if (rotation != 0) {
            type = Type.ROTATE;
            this.velocity = 0;
            angle = SimulationEngine.normalizeAngle(rotation);
        } else {
            type = Type.STAY;
            this.velocity = 0;
            angle = 0;
        }
    }

    @Override
    public boolean equals(Object o) {
        if (o == this) return true;

        if (!(o instanceof Action)) {
            return false;
        }
        Action act = (Action) o;
        return velocity == act.velocity &&
                angle == act.angle &&
                degreeOfRealization == act.degreeOfRealization &&
                type == act.type;
    }

    @Override
    public int hashCode() {
        return Objects.hash(velocity, angle, degreeOfRealization, type);
    }

    public enum Type {
        STAY, MOVE, ROTATE
    }
}
