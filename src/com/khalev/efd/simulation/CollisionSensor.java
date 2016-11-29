package com.khalev.efd.simulation;

/**
 * Interface that any collision sensor must implement.
 */
public interface CollisionSensor {

    void receiveSpatialInput(SpatialInput input);
    boolean inputReceived(SpatialInput input);

}
