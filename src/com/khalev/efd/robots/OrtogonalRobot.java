package com.khalev.efd.robots;

import com.khalev.efd.simulation.*;
import cz.cuni.mff.d3s.deeco.annotations.*;
import cz.cuni.mff.d3s.deeco.annotations.Process;
import cz.cuni.mff.d3s.deeco.task.ParamHolder;

/**
 * This robot moves forward till it hits some obstacle; then it rotates to the direction which is opposite to the
 * collision point.
 */
@Component
public class OrtogonalRobot extends DEECoRobot {

    public OrtogonalRobot() {
        wheels = new SimpleWheels();
    }

    @Process
    @PeriodicScheduling(Environment.CYCLE)
    public static void decisionProcess(
            @In("rID") Integer rid,
            @InOut("wheels") ParamHolder<SimpleWheels> wheels,
            @In("sensor") SensorySystem sensor
    ) {
        Object o = sensor.getInputFromSensor("collisions");
        CollisionData input = (o instanceof CollisionData ? (CollisionData) o : null);
        if (input != null && input.action.type != Action.Type.ROTATE && input.collisionPoints.size() == 1) {
            Environment.getInstance().logger.fine("Robot "+rid+" noticed collision");
            wheels.value.speed = 0.0;
            wheels.value.rotationAngle = input.collisionPoints.get(0) + Math.PI;
        } else {
            wheels.value.speed = 1.0;
            wheels.value.rotationAngle = 0.0;
        }
    }
}
