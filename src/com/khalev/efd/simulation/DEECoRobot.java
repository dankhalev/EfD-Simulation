package com.khalev.efd.simulation;

/**
 * A template for robots. Each robot in simulation must be descendant of this class.
 */
public class DEECoRobot {

    public Integer rID;
    public SensorySystem sensor;
    public Wheels wheels;

    public DEECoRobot() {
        sensor = new SensorySystem();
        sensor.registerSensor("collisions", new Sensor<CollisionData>());

        wheels = new Wheels() {
            @Override
            public Action sendCurrentAction() {
                return new Action(0,0);
            }
        };
    }


}
