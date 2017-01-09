package com.khalev.efd.simulation;

/**
 * This class represents a placement of a robot in the environment. Placement consists of robot's coordinates and
 * current angle of rotation.
 */
public class RobotPlacement {

    DEECoRobot robot;

    Double x;
    Double y;
    Double angle;
    final int id;
    private static int RID = 0;

    RobotPlacement(DEECoRobot robot, Double x, Double y, Double angle) {
        this.robot = robot;
        this.x = x;
        this.y = y;
        this.angle = angle;
        this.id = RID++;
    }

    private RobotPlacement(int id) {
        this.id = id;
    }

    RobotPlacement copy() {
        RobotPlacement rp = new RobotPlacement(id);
        rp.x = x;
        rp.y = y;
        rp.robot = robot;
        rp.angle = angle;
        return rp;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getAngle() {
        return angle;
    }

}
