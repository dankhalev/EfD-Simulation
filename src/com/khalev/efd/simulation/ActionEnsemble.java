package com.khalev.efd.simulation;


import cz.cuni.mff.d3s.deeco.annotations.*;
import cz.cuni.mff.d3s.deeco.task.ParamHolder;

/**
 * Ensemble that communicates robots' actions to the environment.
 */
@Ensemble
@PeriodicScheduling(Environment.CYCLE)
public class ActionEnsemble {

    @Membership
    public static boolean membership(
            @In("coord.phase") Coordinator.Phase phase,
            @In("member.rID") Integer rid,
            @In("member.simulationTime") Integer time,
            @In("member.cyclesCounted") Integer cycle,
            @In("coord.cycle") Integer globalCycle,
            @In("member.wheels") Wheels wheels,
            @In("coord.actions") Action[] actions,
            @In("coord.actionReceived") Boolean[] received,
            @In("coord.actTrigger") Integer trigger
    ) {
        return true;
    }

    @KnowledgeExchange
    public static void map(
            @In("coord.phase") Coordinator.Phase phase,
            @In("member.rID") Integer rid,
            @In("member.simulationTime") Integer time,
            @In("member.cyclesCounted") Integer cycle,
            @In("coord.cycle") Integer globalCycle,
            @In("member.wheels") Wheels wheels,
            @InOut("coord.actions") ParamHolder<Action[]> actions,
            @InOut("coord.actionReceived") ParamHolder<Boolean[]> received,
            @InOut("coord.actTrigger") ParamHolder<Integer> trigger
    ) {
        if (phase.equals(Coordinator.Phase.FETCHING) && time.equals(cycle) && cycle.equals(globalCycle)) {
            if (!received.value[rid]) {
                actions.value[rid] = wheels.sendCurrentAction();
                received.value[rid] = true;
            }
        }
        if (Coordinator.andAll(received.value)) {
            trigger.value++;
        }
    }
}
