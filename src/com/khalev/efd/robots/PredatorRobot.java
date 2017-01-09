package com.khalev.efd.robots;

import com.khalev.efd.simulation.*;
import cz.cuni.mff.d3s.deeco.annotations.*;
import cz.cuni.mff.d3s.deeco.annotations.Process;
import cz.cuni.mff.d3s.deeco.task.ParamHolder;

import java.util.ArrayList;
import java.util.Collections;

@Component
public class PredatorRobot extends DEECoRobot {

    public Coordinates target = new Coordinates(0,0,0);
    public Coordinates position = new Coordinates(0,0,0);

    public ArrayList<Double> array = new ArrayList<>();

    public PredatorRobot() {
        wheels = new SimpleWheels();
        sensor.registerSensor("coords", new Sensor<Coordinates>());
        Double[] arr = new Double[] {
                7.984093553752103,
                1.4786804015530786,
                30.169450043083145,
                45.158619726179616,
                29.633244089416266,
                56.44767224974389,
                39.497351216552175,
                78.49706893805701,
                38.98425306506562,
                89.51335494840036,
                77.25225624508515,
                57.795144467143885,
                35.26141104781606,
                45.46937728363589,
                43.60162866446585,
                98.33502002015517
        };
        Collections.addAll(array, arr);
    }

    @Process
    @PeriodicScheduling(1)
    public static void decision(
            @InOut("wheels") ParamHolder<SimpleWheels> wheels,
            @In("sensor") SensorySystem sensor,
            @InOut("target") ParamHolder<Coordinates> target,
            @InOut("position") ParamHolder<Coordinates> position,
            @InOut("array") ParamHolder<ArrayList<Double>> array
    ) {
        Object o1 = sensor.getInputFromSensor("collisions");
        CollisionData collisionData = (o1 instanceof CollisionData ? (CollisionData) o1 : null);

        Object o2 = sensor.getInputFromSensor("coords");
        Coordinates coordinates = (o2 instanceof Coordinates ? (Coordinates) o2 : null);
        position.value = coordinates;
        if (newTargetRequested(collisionData, coordinates, target.value) && collisionData != null
                && collisionData.action.type != Action.Type.ROTATE
                && wheels.value.rotationAngle == 0.0) {
            target.value = setNewTarget(array.value);
            rotateToTarget(wheels, coordinates, target.value);
        } else if (collisionData != null && collisionData.action.type == Action.Type.ROTATE) {
            wheels.value.speed = 1.0;
            wheels.value.rotationAngle = 0.0;
        }
    }

    private static void rotateToTarget(ParamHolder<SimpleWheels> wheels, Coordinates coordinates, Coordinates target) {
        wheels.value.rotationAngle = Math.atan2(target.x - coordinates.x, target.y - coordinates.y) - coordinates.angle;
        wheels.value.speed = 0.0;
    }

    private static Coordinates setNewTarget(ArrayList<Double> array) {
        //double x = Math.random() * 100;
        //double y = Math.random() * 100;
        double x = array.remove(0);
        double y = array.remove(0);
        return new Coordinates(x, y, 0);
    }

    private static boolean newTargetRequested(CollisionData collisionData, Coordinates coordinates, Coordinates target) {

        double xDist = (coordinates.x - target.x);
        double yDist = (coordinates.y - target.y);
        double distance =  xDist*xDist + yDist*yDist;

        return (distance < 3.0 || collisionData.collisionPoints.size() > 0);
    }

}
