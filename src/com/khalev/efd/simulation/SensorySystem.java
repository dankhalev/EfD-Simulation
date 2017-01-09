package com.khalev.efd.simulation;

import java.util.HashMap;

//TODO: add more protection from illegal use
public class SensorySystem {

    private HashMap<String, Sensor> sensors = new HashMap<>();

    void receiveInput(String name, Object input) {
        Sensor sensor = sensors.get(name);
        sensor.receiveInput(input);
    }

    boolean inputReceived(String name, Object input) {
        Sensor sensor = sensors.get(name);
        return (sensor == null || sensor.inputReceived(input));
    }

    public Object getInputFromSensor(String name) {
        Sensor sensor = sensors.get(name);
        return sensor == null ? null : sensor.getInput();
    }

    public void registerSensor(String name, Sensor sensor) {
        sensors.put(name, sensor);
    }

}
