package com.khalev;

/**
 * A template for robots. Each robot in simulation must be descendant of this class.
 */
public class DEECoRobot {

    public Integer rID;
    public Integer simulationTime = 0;
    public Integer cyclesCounted;
    public CollisionSensor sensor;
    public Wheels wheels;

    public DEECoRobot() {
        sensor = new CollisionSensor() {
            @Override
            public void receiveSpatialInput(SpatialInput input) {

            }

            @Override
            public boolean inputReceived(SpatialInput input) {
                return true;
            }
        };

        cyclesCounted = -1;
        wheels = new Wheels() {
            @Override
            public Action sendCurrentAction() {
                return new Action(0,0);
            }
        };
    }


}
