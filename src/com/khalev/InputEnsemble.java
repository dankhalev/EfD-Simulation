package com.khalev;

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
            @In("coord.cycle") Integer globalCycle,
            @In("member.rID") Integer rid,
            @In("member.simulationTime") Integer cycle,
            @In("member.sensor") CollisionSensor sensor,
            @In("coord.inputs") SpatialInput[] inputs,
            @In("coord.inputSent") Boolean[] sent,
            @In("coord.inTrigger") Integer trigger
    ) {
        return true;
    }

    @KnowledgeExchange
    public static void map(
            @In("coord.phase") Coordinator.Phase phase,
            @In("coord.cycle") Integer globalCycle,
            @In("member.rID") Integer rid,
            @InOut("member.simulationTime") ParamHolder<Integer> cycle,
            @InOut("member.sensor") ParamHolder<CollisionSensor> sensor,
            @InOut("coord.inputs") ParamHolder<SpatialInput[]> inputs,
            @InOut("coord.inputSent") ParamHolder<Boolean[]> sent,
            @InOut("coord.inTrigger") ParamHolder<Integer> trigger
    ) {

        if (phase.equals(Coordinator.Phase.SENDING)) {
            if (!sensor.value.inputReceived(inputs.value[rid])) {
                sensor.value.receiveSpatialInput(inputs.value[rid]);
            }
            if (sensor.value.inputReceived(inputs.value[rid]) && cycle.value.equals(globalCycle) &&!sent.value[rid]) {
                sent.value[rid] = true;
            }
            if (sensor.value.inputReceived(inputs.value[rid]) && !cycle.value.equals(globalCycle)) {
                cycle.value = cycle.value + 1;
            }

        }
        if (Coordinator.andAll(sent.value)) {
            trigger.value++;
        }
    }
}
