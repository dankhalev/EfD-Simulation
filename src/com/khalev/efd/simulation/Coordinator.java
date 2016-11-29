package com.khalev.efd.simulation;

import cz.cuni.mff.d3s.deeco.annotations.*;
import cz.cuni.mff.d3s.deeco.annotations.Process;
import cz.cuni.mff.d3s.deeco.task.ParamHolder;

/**
 * DEECo Component that works directly with the environment. It collects actions from all the robots
 * (using ActionEnsemble), triggers next cycle in the environment and receives inputs for robots' sensors.
 * Those inputs are later distributed to robots using InputEnsemble.
 */
@Component
public class Coordinator {

    public Coordinator.Phase phase = Phase.FETCHING;
    public Action[] actions;
    public Boolean[] actionReceived;
    public SpatialInput[] inputs;
    public Boolean[] inputSent;
    public Integer numOfRobots;

    public Integer actTrigger = 0;
    public Integer inTrigger = 0;
    public Integer cycle = 0;


    public Coordinator(int numOfRobots) {
        this.numOfRobots = numOfRobots;
        actions = new Action[numOfRobots];
        actionReceived = new Boolean[numOfRobots];
        inputs = new SpatialInput[numOfRobots];
        inputSent = new Boolean[numOfRobots];
        for (int i = 0; i < numOfRobots; i++) {
            actionReceived[i] = false;
            inputSent[i] = false;

        }
    }

    @Process
    public static void nextCycle(
        @InOut("actions") ParamHolder<Action[]> actions,
        @InOut("actionReceived") ParamHolder<Boolean[]> received,
        @InOut("inputs") ParamHolder<SpatialInput[]> inputs,
        @InOut("inputSent") ParamHolder<Boolean[]> sent,
        @InOut("phase") ParamHolder<Phase> phase,
        @In("numOfRobots") Integer numOfRobots,
        @TriggerOnChange @In("inTrigger") Integer inTrigger,
        @TriggerOnChange @In("actTrigger") Integer actTrigger,
        @InOut("cycle") ParamHolder<Integer> cycle
    ) {
        if (phase.value == Phase.FETCHING && andAll(received.value)) {
            phase.value = Phase.PROCESSING;
            Environment env = Environment.getInstance();
            env.updateActions(actions.value);
            env.actualizedActions = true;

            env.cycle();
            env.logger.fine("Cycle: " + cycle.value);
            cycle.value++;
            inputs.value = env.returnInputs();
            env.actualizedSpatialInputs = false;
            for (int i = 0; i < numOfRobots; i++) {
                sent.value[i] = false;
                received.value[i] = false;
            }
            phase.value = Phase.SENDING;
            env.logger.fine("SENDING PHASE");
        } else if (andAll(sent.value) && phase.value != Phase.FETCHING) {
            phase.value = Phase.FETCHING;
            Environment.getInstance().logger.fine("FETCHING PHASE");
        }
    }



    public static boolean andAll(Boolean[] f) {
        for (boolean b : f) {
            if (!b)
                return false;
        }
        return true;
    }

    enum Phase {
        FETCHING, PROCESSING, SENDING
    }

}
