package com.khalev.efd.robots;

import com.khalev.efd.simulation.*;
import cz.cuni.mff.d3s.deeco.annotations.*;
import cz.cuni.mff.d3s.deeco.annotations.Process;
import cz.cuni.mff.d3s.deeco.task.ParamHolder;

/**
 * This robot moves forward till it hits some obstacle; then, it changes its direction to opposite.
 */
@Component
public class SimpleRobot extends DEECoRobot {

    public SimpleRobot() {
        sensor = new SimpleCollisionSensor();
        wheels = new SimpleWheels();
    }

    @Process
    @PeriodicScheduling(Environment.CYCLE)
    public static void decisionProcess(
            @In("rID") Integer rid,
            @InOut("wheels") ParamHolder<SimpleWheels> wheels,
            @In("sensor") SimpleCollisionSensor sensor
    ) {
        if (!sensor.input.collisionPoints.isEmpty() && sensor.input.action.type != Action.Type.ROTATE) {
            wheels.value.speed = 0.0;
            wheels.value.rotationAngle = Math.PI;
        } else {
            wheels.value.speed = 1.0;
            wheels.value.rotationAngle = 0.0;
        }
    }
}
