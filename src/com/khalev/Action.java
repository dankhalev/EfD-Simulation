package com.khalev;

/**
 * This class represents an action that a robot can perform in a cycle of simulation.
 */
public class Action {

    public static final double MAX_SPEED = 1.0;


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
            angle = Environment.normalizeAngle(rotation);
        } else {
            type = Type.STAY;
            this.velocity = 0;
            angle = 0;
        }
    }

    enum Type {
        STAY, MOVE, ROTATE
    }
}
