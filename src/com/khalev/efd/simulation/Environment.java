package com.khalev.efd.simulation;

import cz.cuni.mff.d3s.deeco.runtime.RuntimeFramework;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.logging.*;

/**
 * This class is used for computing cycles of the simulation and for writing logs.
 */
public class Environment {

    public static final int CYCLE = 2;
    private final int STEPS;

    private RuntimeFramework runtime;
    private EnvironmentMap environmentMap;
    private SimulationEngine computer;
    private ArrayList<RobotPlacement> robots;
    private ArrayList<Action> actions = new ArrayList<>();
    private ArrayList<SpatialInput> inputs = new ArrayList<>();

    private int step;
    private int previousStep = -1;
    private FileWriter logfile;
    private long startTime;
    Logger logger;

    private static Environment instance;
    public static Environment getInstance() {
        return instance;
    }
    public static void setInstance(Environment e) {
        instance = e;
    }

    public Environment(int numOfSteps, ArrayList<RobotPlacement> robots, File logs, EnvironmentMap map, String bitmap) {
        STEPS = numOfSteps;
        try {
            this.environmentMap = map;
            this.robots = robots;
            this.computer = new SimulationEngine(robots, map);
            this.logfile = new FileWriter(logs);
            logfile.write(bitmap + "\n");

            for ( int i = 0; i < robots.size(); i++) {
                robots.get(i).robot.rID = i;
            }
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public void setRuntime(RuntimeFramework runtime) {
        configureLogger();
        this.runtime = runtime;
    }

    private void configureLogger() {
        logger = Logger.getLogger(this.getClass().getPackage().getName());
        LogManager.getLogManager().reset();
        Formatter formatter = new Formatter() {
            @Override
            public String format(LogRecord arg0) {
                StringBuilder b = new StringBuilder();
                b.append(arg0.getSourceMethodName()).append("(): ");
                b.append(arg0.getMessage());
                b.append(System.getProperty("line.separator"));
                return b.toString();
            }
        };

        Handler ch = new ConsoleHandler();
        ch.setFormatter(formatter);
        ch.setLevel(Level.INFO);
        logger.addHandler(ch);
        LogManager lm = LogManager.getLogManager();
        lm.addLogger(logger);
        logger.setLevel(Level.INFO);

        startTime = System.nanoTime();
    }

    public void updateActions(Action[] acts) {
        actions.clear();
        assert acts.length == robots.size(): "Received Action[] array has wrong size";
        Collections.addAll(actions, acts);
    }

    public SpatialInput[] returnInputs() {
        assert inputs.size() == robots.size(): "Number of robots and number of inputs are not the same";
        SpatialInput[] sis = new SpatialInput[inputs.size()];
        for (int i = 0; i < sis.length; i++) {
            sis[i] = inputs.get(i);
        }
        return sis;
    }

    //TODO: find better way to exit the program
    public int cycle() {
        try {
            writeLogs();
            logger.info("CYCLE " + step);
            assert step == previousStep + 1: "Unknown exception occurred in computations";
            previousStep++;
            if (!endCondition()) {
                robots = computer.performActions(actions);
                inputs = computer.sendInputs(robots);
                step++;
                return 0;
            } else {
                writeTimerLogs();
                logfile.close();
                this.runtime.stop();
                System.exit(0);
                return 1;
            }
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    private void writeTimerLogs() {
        long endTime = System.nanoTime();
        logger.info("Time elapsed: " + ((endTime - startTime) / 1000000) + " ms");
        logger.info("Approximately " + ((endTime - startTime) / this.STEPS / 1000000) + " ms per cycle");
    }

    private void writeLogs() throws IOException {
        for (RobotPlacement robot : robots) {
            logfile.write(robot.x.toString());
            logfile.write(",");
            logfile.write(robot.y.toString());
            logfile.write(";");
        }
        logfile.write("\n");
    }

    private boolean endCondition() {
        return step >= STEPS;
    }
}
