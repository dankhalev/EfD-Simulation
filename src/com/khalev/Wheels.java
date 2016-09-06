package com.khalev;

/**
 * Interface for robot's wheels. Wheels are a part of the robot that signalizes current action to environment.
 */
public interface Wheels {

    Action sendCurrentAction();

}
