package com.khalev.efd.simulation;

import cz.cuni.mff.d3s.deeco.annotations.*;
import cz.cuni.mff.d3s.deeco.task.ParamHolder;

import java.util.ArrayList;

/**
 * Ensemble that is responsible for transitioning of sensory input from environment to robots.
 */
@Ensemble
@PeriodicScheduling(Environment.CYCLE)
public class InputEnsemble {

    @Membership
    public static boolean membership(
            @In("coord.phase") Coordinator.Phase phase,
            @In("member.rID") Integer rid,
            @In("member.sensor") SensorySystem sensor,
            @In("coord.sensorNames") ArrayList<String> names,
            @In("coord.inputs") CollisionData[] inputs,
            @In("coord.allInputs") ArrayList<ArrayList> allInputs,
            @In("coord.inputSent") Boolean[] sent
    ) {
        return true;
    }

    @KnowledgeExchange
    public static void map(
            @In("coord.phase") Coordinator.Phase phase,
            @In("member.rID") Integer rid,
            @InOut("member.sensor") ParamHolder<SensorySystem> sensor,
            @In("coord.sensorNames") ArrayList<String> names,
            @InOut("coord.inputs") ParamHolder<CollisionData[]> inputs,
            @In("coord.allInputs") ArrayList<ArrayList> allInputs,
            @InOut("coord.inputSent") ParamHolder<Boolean[]> sent
    ) {
        if (phase.equals(Coordinator.Phase.SENDING)) {
            boolean b = true;
            for (int i = 0; i < names.size(); i++) {
                if (!sensor.value.inputReceived(names.get(i), allInputs.get(i).get(rid))) {
                    sensor.value.receiveInput(names.get(i), allInputs.get(i).get(rid));
                    b = false;
                } else {
                    if (inputs.value[rid].collisionPoints.size() > 0) {
                        Environment.getInstance().logger.fine("Robot "+rid+" received collision");
                    }
                }
            }
            sent.value[rid] = b;
        }
    }
}
