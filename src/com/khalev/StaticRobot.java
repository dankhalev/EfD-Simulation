package com.khalev;

import cz.cuni.mff.d3s.deeco.annotations.*;
import cz.cuni.mff.d3s.deeco.annotations.Process;
import cz.cuni.mff.d3s.deeco.task.ParamHolder;

/**
 * This robot always holds its position, never moves.
 */
@Component
public class StaticRobot extends DEECoRobot {

    @Process
    @PeriodicScheduling(Environment.CYCLE / 2)
    public static void decisionProcess(
            @In("simulationTime") Integer cycle,
            @InOut("cyclesCounted") ParamHolder<Integer> count
    ) {
        count.value = cycle;
    }


}
