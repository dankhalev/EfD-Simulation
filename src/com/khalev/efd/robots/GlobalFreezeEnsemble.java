package com.khalev.efd.robots;

import cz.cuni.mff.d3s.deeco.annotations.*;
import cz.cuni.mff.d3s.deeco.task.ParamHolder;

@Ensemble
@PeriodicScheduling(1)
public class GlobalFreezeEnsemble {

    @Membership
    public static boolean membership(
            @In("member.freeze") Boolean freeze,
            @In("coord.cycle") Integer cycle,
            @In("member.rID") Integer rid,
            @In("member.wheels") SimpleWheels wheels
    ) {
        return true;
    }

    @KnowledgeExchange
    public static void map(
            @In("coord.cycle") Integer cycle,
            @In("member.rID") Integer rid,
            @InOut("member.wheels") ParamHolder<SimpleWheels> wheels,
            @InOut("member.freeze") ParamHolder<Boolean> freeze
    ) {
        if (cycle % 3 != 0 && rid % 2 != 0) {
            wheels.value.speed = 0.0;
            freeze.value = true;
        } else {
            freeze.value = false;
        }
    }
}
