package com.khalev.efd.simulation;

import java.util.ArrayList;

public abstract class SensoryInputsProcessor {

    EnvironmentMap environmentMap;

    public SensoryInputsProcessor(EnvironmentMap map) {
        this.environmentMap = map;
    }

    abstract Object sendInputs(ArrayList<RobotPlacement> robots);
}
