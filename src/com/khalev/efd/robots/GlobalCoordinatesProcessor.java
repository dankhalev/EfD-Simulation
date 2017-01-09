package com.khalev.efd.robots;

import com.khalev.efd.simulation.EnvironmentMap;
import com.khalev.efd.simulation.RobotPlacement;
import com.khalev.efd.simulation.SensoryInputsProcessor;

import java.util.ArrayList;

public class GlobalCoordinatesProcessor extends SensoryInputsProcessor<Coordinates> {

    public GlobalCoordinatesProcessor(EnvironmentMap map) {
        super(map);
    }

    @Override
    protected ArrayList<Coordinates> sendInputs(ArrayList<RobotPlacement> robots) {
        ArrayList<Coordinates> coords = new ArrayList<>();
        for (RobotPlacement r : robots) {
            coords.add(new Coordinates(r.getX(), r.getY(), r.getAngle()));
        }
        return coords;
    }
}
