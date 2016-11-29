package com.khalev.efd.simulation;

/**
 * Any exception that is caused by inconsistency of simulation's initial parameters.
 */
public class SimulationParametersException extends Exception {
    public SimulationParametersException(String s) {
        super(s);
    }

    public SimulationParametersException(String s, Throwable throwable) {
        super(s, throwable);
    }
}
