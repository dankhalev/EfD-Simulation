package com.khalev.efd.simulation;

import cz.cuni.mff.d3s.deeco.runtime.RuntimeFramework;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.logging.*;

//TODO: Separate logging from Environment. Consider writing logfile & runtime logs in separate objects.
//TODO: Remove old-style ArrayList<CollisionData> inputs from Environment, Coordinator & InputEnsemble
//TODO: Add protection from external sensors changing positions of robots
/**
 * This class is used for computing cycles of the simulation and for writing logs.
 */
public class Environment {

    public static final int CYCLE = 1;
    private final int STEPS;

    private RuntimeFramework runtime;
    private EnvironmentMap environmentMap;
    private SimulationEngine computer;
    private ArrayList<RobotPlacement> robots;
    private ArrayList<RobotPlacement> previousPositions;
    private ArrayList<Action> actions = new ArrayList<>();
    private ArrayList<CollisionData> inputs = new ArrayList<>();

    private ArrayList<ArrayList> allInputs;
    private ArrayList<SensoryInputsProcessor> sensors;

    private int step;
    private int previousStep = -1;
    private FileWriter logfile;
    private long startTime;
    public Logger logger;

    private static Environment instance;
    public static Environment getInstance() {
        return instance;
    }
    static void setInstance(Environment e) {
        instance = e;
    }

    Environment(int numOfSteps, ArrayList<RobotPlacement> robots, File logs, EnvironmentMap map, String bitmap,
                ArrayList<SensoryInputsProcessor> sensors) {
        try {
            STEPS = numOfSteps;
            this.environmentMap = map;
            this.robots = robots;
            this.computer = new SimulationEngine(robots, map);

            this.sensors = new ArrayList<>();
            this.sensors.add(this.computer);
            this.sensors.addAll(sensors);
            this.allInputs = new ArrayList<>(sensors.size());

            this.logfile = new FileWriter(logs);
            logfile.write(bitmap + "\n");
            for ( int i = 0; i < robots.size(); i++) {
                robots.get(i).robot.rID = i;
            }
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    void setRuntime(RuntimeFramework runtime) {
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

    void updateActions(Action[] acts) {
        actions.clear();
        assert acts.length == robots.size(): "Received Action[] array has wrong size";
        Collections.addAll(actions, acts);
    }

    CollisionData[] returnInputs() {
        assert inputs.size() == robots.size(): "Number of robots and number of inputs are not the same";
        CollisionData[] sis = new CollisionData[inputs.size()];
        for (int i = 0; i < sis.length; i++) {
            sis[i] = inputs.get(i);
        }
        return sis;
    }

    ArrayList<ArrayList> getAllInputs() {
        return allInputs;
    }

    //TODO: find better way to exit the program
    int cycle() {
        try {
            writeLogs();
            logger.info("CYCLE " + step);
            assert step == previousStep + 1: "Unknown exception occurred in computations";
            previousStep++;

            if (!endCondition()) {
                previousPositions = robots;
                robots = computer.performActions(actions);
                allInputs.clear();
                for (SensoryInputsProcessor sip : sensors) {
                    allInputs.add(sip.sendInputs(robots));
                }
                @SuppressWarnings("unchecked")
                ArrayList<CollisionData> list = allInputs.get(0);
                inputs = list;
                if (step > 0)
                    writeRobotLogs();
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

    private void writeRobotLogs() {
        String s = "\n";
        for (int i = 0; i < previousPositions.size(); i++) {
            RobotPlacement r = robots.get(i);
            RobotPlacement rp = previousPositions.get(i);
            Action act = actions.get(i);
            CollisionData in = inputs.get(i);

            s += ("Robot #" + rp.id + " had coordinates " + rp.x + ", " + rp.y + ";\n");
            if (act.type == Action.Type.MOVE) {
                s += ("Robot #" + r.id + ": MOVE, " + ((act.degreeOfRealization)*100) + "%\n");
            } else if (act.type == Action.Type.ROTATE) {
                s += ("Robot #" + r.id + ": ROTATE, " + (Math.toDegrees(act.angle)) + " degrees\n");
            } else {
                s += ("Robot #" + r.id + ": STAY\n");
            }
            if (in.collisionPoints.size() == 0) {
                s += ("Robot #" + r.id + ": no collisions\n");
            } else {
                s += ("Robot #" + r.id + ": COLLISIONS AT ");
                for (double point : in.collisionPoints) {
                    s += String.format("%.2f", Math.toDegrees(point));
                    s += ("; ");
                }
                s += "\n";
            }
            s += ("Robot #" + r.id + " has final coordinates " + r.x + ", " + r.y + ";\n");
        }
        logger.finer(s);
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
