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
            @In("member.wheels") Wheels wheels,
            @In("coord.actions") Action[] actions,
            @In("coord.actionReceived") Boolean[] received
    ) {
        return true;
    }

    @KnowledgeExchange
    public static void map(
            @In("coord.phase") Coordinator.Phase phase,
            @In("member.rID") Integer rid,
            @In("member.wheels") Wheels wheels,
            @InOut("coord.actions") ParamHolder<Action[]> actions,
            @InOut("coord.actionReceived") ParamHolder<Boolean[]> received
    ) {
        if (phase.equals(Coordinator.Phase.FETCHING)) {
            if (!received.value[rid]) {
                actions.value[rid] = wheels.sendCurrentAction();
                received.value[rid] = true;
                if (actions.value[rid].type == Action.Type.ROTATE) {
                    Environment.getInstance().logger.fine("Robot "+rid+" noticed collision indeed");
                }
            }
        }
    }
}
