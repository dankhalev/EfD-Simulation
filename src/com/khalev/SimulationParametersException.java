package com.khalev;

/**
 * Any exception that is caused by inconsistency of simulation's initial parameters.
 */
public class SimulationParametersException extends Exception {
    public SimulationParametersException(String s) {
        super(s);
        System.out.println(s);
    }

    public SimulationParametersException(String s, Throwable throwable) {
        super(s, throwable);
        System.out.println(s);
    }
}
