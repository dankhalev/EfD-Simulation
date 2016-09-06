package com.khalev;

import cz.cuni.mff.d3s.deeco.annotations.*;
import cz.cuni.mff.d3s.deeco.annotations.Process;
import cz.cuni.mff.d3s.deeco.task.ParamHolder;

import java.util.ArrayList;
import java.util.Objects;

/**
 * This robot moves forward till it hits some obstacle; then, it changes its direction to opposite.
 */
@Component
public class SimpleRobot extends DEECoRobot {


    public SimpleRobot() {
        simulationTime = 0;
        cyclesCounted = -1;
        sensor = new CollisionSensor();
        wheels = new Wheels();
    }

    @Process
    @PeriodicScheduling(Environment.CYCLE / 2)
    public static void decisionProcess(
            @In("rID") Integer rid,
            @InOut("wheels") ParamHolder<Wheels> wheels,
            @In("sensor") CollisionSensor sensor,
            @In("simulationTime") Integer cycle,
            @InOut("cyclesCounted") ParamHolder<Integer> count
    ) {
        if (!sensor.collisionPoints.isEmpty() && wheels.value.rotationAngle == 0.0 && !cycle.equals(count.value)) {
            wheels.value.speed = 0.0;
            wheels.value.rotationAngle = Math.PI;
            count.value++;
            //System.out.println("Robot: " + rid + " Cycle: " + cycle + " Count " + count.value + " Decision: ROTATE");
        } else if (!cycle.equals(count.value)) {
            wheels.value.speed = 1.0;
            wheels.value.rotationAngle = 0.0;
            count.value++;
            //System.out.println("Robot: " + rid + " Cycle: " + cycle + " Count " + count.value + " Decision: MOVE");
        }

    }



    class CollisionSensor implements com.khalev.CollisionSensor {
        ArrayList<Double> collisionPoints = new ArrayList<>();

        public void receiveSpatialInput(SpatialInput input) {
            this.collisionPoints = input.collisionPoints;

        }

        public boolean inputReceived(SpatialInput input) {

            if (this.collisionPoints.size() != input.collisionPoints.size()) {
                return false;
            } else for (int i = 0; i < this.collisionPoints.size(); i++) {
                if (!Objects.equals(this.collisionPoints.get(i), input.collisionPoints.get(i))) {
                    return false;
                }
            }
                return true;
        }

    }

    class Wheels implements com.khalev.Wheels {
        public static final double MAX_SPEED = 1.0;
        double rotationAngle = 0.0;
        double speed = 1.0;

        public Action sendCurrentAction() {
            return new Action(this.speed * MAX_SPEED, this.rotationAngle);
        }
    }
}