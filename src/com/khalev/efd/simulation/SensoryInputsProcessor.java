package com.khalev.efd.simulation;

import java.util.ArrayList;

public abstract class SensoryInputsProcessor<T> {

    private EnvironmentMap environmentMap;

    public SensoryInputsProcessor(EnvironmentMap map) {
        this.environmentMap = map;
    }

    protected abstract ArrayList<T> sendInputs(ArrayList<RobotPlacement> robots);
}
