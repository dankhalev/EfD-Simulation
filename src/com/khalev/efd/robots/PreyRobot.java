package com.khalev.efd.robots;

import com.khalev.efd.simulation.*;
import cz.cuni.mff.d3s.deeco.annotations.*;
import cz.cuni.mff.d3s.deeco.annotations.Process;
import cz.cuni.mff.d3s.deeco.task.ParamHolder;

@Component
public class PreyRobot extends DEECoRobot {

    public Coordinates predator = new Coordinates(0,0,0);
    public Coordinates position = new Coordinates(0,0,0);
    public Boolean alert = false;
    public Boolean reaction = false;

    public PreyRobot() {
        wheels = new SimpleWheels();
        sensor.registerSensor("coords", new Sensor<Coordinates>());
    }

    @Process
    @PeriodicScheduling(1)
    public static void decision(
            @InOut("wheels") ParamHolder<SimpleWheels> wheels,
            @In("sensor") SensorySystem sensor,
            @InOut("alert") ParamHolder<Boolean> alert,
            @InOut("reaction") ParamHolder<Boolean> reaction,
            @InOut("predator") ParamHolder<Coordinates> predator,
            @InOut("position") ParamHolder<Coordinates> position
    ) {
        Object o1 = sensor.getInputFromSensor("collisions");
        CollisionData collisionData = (o1 instanceof CollisionData ? (CollisionData) o1 : null);

        Object o2 = sensor.getInputFromSensor("coords");
        Coordinates coordinates = (o2 instanceof Coordinates ? (Coordinates) o2 : null);
        if (coordinates != null) {
            position.value = coordinates;
        }

        if (alert.value && (!reaction.value || collisionData.action.type == Action.Type.MOVE)) {
            runAway(coordinates, predator.value, wheels);
            reaction.value = true;
        } else if (alert.value && reaction.value && collision(collisionData)) {
            rotate(collisionData, wheels);
        } else if (!alert.value) {
            reaction.value = false;
            calm(wheels);
        } else {
            wheels.value.rotationAngle = 0.0;
            wheels.value.speed = 1.0;
        }
    }

    private static void runAway(Coordinates position, Coordinates predator, ParamHolder<SimpleWheels> wheels) {
        wheels.value.rotationAngle = Math.atan2(predator.x - position.x, predator.y - position.y) - position.angle + Math.PI;
        wheels.value.speed = 0.0;
    }

    private static void rotate(CollisionData collisionData, ParamHolder<SimpleWheels> wheels) {
        if (collisionData.action.type != Action.Type.ROTATE) {
            wheels.value.speed = 0.0;
            wheels.value.rotationAngle = Math.PI / 3.0;
        } else {
            wheels.value.rotationAngle = 0.0;
            wheels.value.speed = 1.0;
        }
    }

    private static void calm(ParamHolder<SimpleWheels> wheels) {
        wheels.value.rotationAngle = 0.0;
        wheels.value.speed = 0.0;
    }

    private static boolean collision(CollisionData collisionData) {
        return collisionData != null && collisionData.collisionPoints.size() > 0;
    }
}
