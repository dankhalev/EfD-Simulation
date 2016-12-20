package com.khalev.efd.simulation;

import cz.cuni.mff.d3s.deeco.annotations.*;
import cz.cuni.mff.d3s.deeco.task.ParamHolder;

/**
 * Ensemble that is responsible for transitioning of spatial input from environment to robots.
 */
@Ensemble
@PeriodicScheduling(Environment.CYCLE)
public class InputEnsemble {

    @Membership
    public static boolean membership(
            @In("coord.phase") Coordinator.Phase phase,
            @In("member.rID") Integer rid,
            @In("member.sensor") CollisionSensor sensor,
            @In("coord.inputs") SpatialInput[] inputs,
            @In("coord.inputSent") Boolean[] sent
    ) {
        return true;
    }

    @KnowledgeExchange
    public static void map(
            @In("coord.phase") Coordinator.Phase phase,
            @In("member.rID") Integer rid,
            @InOut("member.sensor") ParamHolder<CollisionSensor> sensor,
            @InOut("coord.inputs") ParamHolder<SpatialInput[]> inputs,
            @InOut("coord.inputSent") ParamHolder<Boolean[]> sent
    ) {
        if (phase.equals(Coordinator.Phase.SENDING)) {
            if (sensor.value.inputReceived(inputs.value[rid])) {
                if (inputs.value[rid].collisionPoints.size() > 0) {
                    Environment.getInstance().logger.fine("Robot "+rid+" received collision");
                }
                sent.value[rid] = true;
            }
            if (!sensor.value.inputReceived(inputs.value[rid])) {
                sensor.value.receiveSpatialInput(inputs.value[rid]);
            }
        }
    }
}
