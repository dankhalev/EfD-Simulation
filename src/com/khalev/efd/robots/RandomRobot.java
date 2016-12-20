package com.khalev.efd.robots;

import com.khalev.efd.simulation.*;
import cz.cuni.mff.d3s.deeco.annotations.*;
import cz.cuni.mff.d3s.deeco.annotations.Process;
import cz.cuni.mff.d3s.deeco.task.ParamHolder;

/**
 * This robot moves forward with random speed till it hits some obstacle; then, it rotates a random degree.
 * It can also rotate at any other moment with probability of 2%.
 */
@Component
public class RandomRobot extends DEECoRobot {

    public RandomRobot() {
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
            wheels.value.rotationAngle = Math.PI * Math.random();
        } else {
            if (Math.random() < 0.998) {
                wheels.value.speed = 1.0 * Math.random();
                wheels.value.rotationAngle = 0.0;
            } else {
                wheels.value.speed = 0.0;
                wheels.value.rotationAngle = Math.PI * Math.random();
            }
        }
    }
}
