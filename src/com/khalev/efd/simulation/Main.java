package com.khalev.efd.simulation;

import cz.cuni.mff.d3s.deeco.annotations.processor.AnnotationProcessorException;

//TODO: comment all the public elements in the project
public class Main {

    public static void main(String[] args) throws AnnotationProcessorException, SimulationParametersException {
        if (args.length < 1) {
            throw  new SimulationParametersException("Please provide a name of XML file with simulation properties");
        } else {
            Simulation simulation = new Simulation(args[0]);
            simulation.startSimulation();
        }
    }

}
