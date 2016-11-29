package com.khalev.efd.simulation;

import cz.cuni.mff.d3s.deeco.runtime.RuntimeFramework;
import javafx.util.Pair;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Timer;
import java.util.logging.*;

/**
 * This class is used for computing cycles of the simulation and for writing logs.
 */
public class Environment {

    boolean oldMode = false;
    public static final double ROBOT_RADIUS = 3;
    public static final double ROBOT_RADIUS_SQUARED = ROBOT_RADIUS * ROBOT_RADIUS;
    public static final double DOUBLE_RADIUS_SQUARED = (2 * ROBOT_RADIUS) * (2 * ROBOT_RADIUS);
    public static final int CYCLE = 10;
    private static final double ST_ERR = 0.1;
    private static final double ST_ERR_SQR = 0.01;
    private static final int PRECISION = 4;
    private static final double DENOMINATOR = Math.pow(10, PRECISION);

    public EnvironmentMap environmentMap;
    public final int SIZE_X;
    public final int SIZE_Y;
    public final int STEPS;
    private long startTime;
    public Logger logger;
    void configureLogger() {
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

    private int step;
    private FileWriter logfile;

    private static Environment instance;
    public static Environment getInstance() {
        return instance;
    }
    public static void setInstance(Environment e) {
        instance = e;
    }

    private RuntimeFramework runtime;
    public void setRuntime(RuntimeFramework runtime) {
        configureLogger();
        this.runtime = runtime;
    }

    private ArrayList<RobotPlacement> robots;
    private double[][] distances;

    public ArrayList<Action> actions = new ArrayList<>();
    public void updateActions(Action[] acts) {
        actions.clear();
        if (acts.length == robots.size()) {
            Collections.addAll(actions, acts);
        } else {
            throw new ArrayIndexOutOfBoundsException();
        }
    }

    public ArrayList<SpatialInput> inputs = new ArrayList<>();
    public SpatialInput[] returnInputs() {
        if (inputs.size() == robots.size()) {
            SpatialInput[] sis = new SpatialInput[inputs.size()];
            for (int i = 0; i < sis.length; i++) {
                sis[i] = inputs.get(i);
            }
            return sis;
        } else {
            throw new ArrayIndexOutOfBoundsException();
        }
    }

    CollisionList collisionList;

    public boolean actualizedActions = false;
    public boolean actualizedSpatialInputs = false;


    public Environment(int numOfSteps, ArrayList<RobotPlacement> robots, File logs, EnvironmentMap map) {
        SIZE_X = map.sizeX;
        SIZE_Y = map.sizeY;
        environmentMap = map;
        collisionList = new CollisionList(robots.size());
        STEPS = numOfSteps;
        try {
            this.robots = robots;
            this.distances = new double[robots.size()][robots.size()];
            logfile = new FileWriter(logs);
            logfile.write(SIZE_X + "\n");
            logfile.write(SIZE_Y + "\n");

            for ( int i = 0; i < robots.size(); i++) {
                robots.get(i).robot.rID = i;
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public int cycle() {
        writeLogs();
        logger.info("CYCLE " + step);
        if (!endCondition()) {
            performActions();
            sendInputs();
            step++;
            return 0;
        } else {
            try {
                writeTimerLogs();
                logfile.close();
                this.runtime.stop();
                System.exit(0);
                return 1;
            } catch (IOException e) {
                e.printStackTrace();
                this.runtime.stop();
                System.exit(-1);
                return -1;
            }

        }
    }

    private void writeTimerLogs() {
        long endTime = System.nanoTime();
        logger.info("Time elapsed: " + ((endTime - startTime) / 1000000) + " ms");
        logger.info("Approximately " + ((endTime - startTime) / this.STEPS / 1000000) + " ms per cycle");
    }

    private void writeLogs() {
        try {
            for (RobotPlacement robot : robots) {
                logfile.write(robot.x.toString());
                logfile.write(",");
                logfile.write(robot.y.toString());
                logfile.write(";");
            }
            logfile.write("\n");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void sendInputs() {
        inputs.clear();

        for (RobotPlacement rp : robots) {
            inputs.add(new SpatialInput());
        }

        //TO-DO: rewrite this code for general case (with walls inside the field)
    if (oldMode) {
        for (int i = 0; i < robots.size(); i++) {
            RobotPlacement r = robots.get(i);

            if (r.x <= ROBOT_RADIUS) {
                double angle = normalizeAngle(Math.atan2(-3, 0) - r.angle);
                inputs.get(i).collisionPoints.add(angle);
            }
            if (r.x >= SIZE_X - ROBOT_RADIUS) {
                double angle = normalizeAngle(Math.atan2(3, 0) - r.angle);
                inputs.get(i).collisionPoints.add(angle);
            }
            if (r.y <= ROBOT_RADIUS) {
                double angle = normalizeAngle(Math.atan2(0, -3) - r.angle);
                inputs.get(i).collisionPoints.add(subjectiveAngleBetween(r, r.x, 0));
            }
            if (r.y >= SIZE_Y - ROBOT_RADIUS) {
                double angle = normalizeAngle(Math.atan2(0, 3) - r.angle);
                inputs.get(i).collisionPoints.add(angle);
            }
        }
    } else {
        collisionList.computeCollisionsWithWalls(robots, inputs);
    }

        for (int i = 0; i < distances.length; i++) {
            for (int j = 0; j < distances[i].length; j++) {
                distances[i][j] = distance(robots.get(i), robots.get(j));
            }
        }

        for (int i = 0; i < distances.length; i++) {
            for (int j = 0; j < distances[i].length; j++) {
                if (i > j && distances[i][j] < DOUBLE_RADIUS_SQUARED + ST_ERR) {
                    RobotPlacement r1 = robots.get(i);
                    RobotPlacement r2 = robots.get(j);
                    inputs.get(i).collisionPoints.add(subjectiveAngleBetween(r1, r2));
                    inputs.get(j).collisionPoints.add(subjectiveAngleBetween(r2, r1));
                }
            }
        }


        actualizedSpatialInputs = true;
    }

    public static double normalizeAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    private void performActions() {
        // --- Creating a placement list for next step:
        ArrayList<RobotPlacement> next = new ArrayList<>();
        for (RobotPlacement rp : robots) {
            next.add(rp.copy());
        }

        // --- Performing actions:
        for (int i = 0; i < actions.size(); i++) {
            Action act = actions.get(i);
            RobotPlacement place = next.get(i);

            switch (act.type) {
                case ROTATE:
                    place.angle = normalizeAngle(place.angle + act.angle);
                    act.degreeOfRealization = 0;
                    break;
                case STAY:
                    act.degreeOfRealization = 0;
                    break;
                case MOVE:
                    place.x += act.velocity * Math.sin(place.angle);
                    place.y += act.velocity * Math.cos(place.angle);
                    act.degreeOfRealization = 1;
                    break;
            }
        }

        // --- Resolving robot-wall collisions:
    if (oldMode) {
         for (int i = 0; i < robots.size(); i++) {
             RobotPlacement r = next.get(i);
             RobotPlacement rp = robots.get(i);
             Action act = actions.get(i);
             if (r.x < ROBOT_RADIUS) {
                 double t = (ROBOT_RADIUS - rp.x) / (act.velocity * Math.sin(rp.angle));
                 r.x = ROBOT_RADIUS;
                 r.y = rp.y + act.velocity * Math.cos(rp.angle) * t;
                 act.degreeOfRealization = t;
             }
             if (r.x > SIZE_X - ROBOT_RADIUS) {
                 double t = (SIZE_X - ROBOT_RADIUS - rp.x) / (act.velocity * Math.sin(rp.angle));
                 r.x = SIZE_X - ROBOT_RADIUS;
                 r.y = rp.y + act.velocity * Math.cos(rp.angle) * t;
                 act.degreeOfRealization = t;
             }
             if (r.y < ROBOT_RADIUS) {
                 double t = (ROBOT_RADIUS - rp.y) / (act.velocity * Math.cos(rp.angle));
                 r.y = ROBOT_RADIUS;
                 r.x = rp.x + act.velocity * Math.sin(rp.angle) * t;
                 act.degreeOfRealization = t;
             }
             if (r.y > SIZE_Y - ROBOT_RADIUS) {
                 double t = (SIZE_Y - ROBOT_RADIUS - rp.y) / (act.velocity * Math.cos(rp.angle));
                 r.y = SIZE_Y - ROBOT_RADIUS;
                 r.x = rp.x + act.velocity * Math.sin(rp.angle) * t;
                 act.degreeOfRealization = t;
             }
         }
    } else {
         ArrayList<Integer> wallCollisionList = new ArrayList<>();
         for (int i = 0; i < robots.size(); i++) {
             wallCollisionList.add(i);
         }

         while (!wallCollisionList.isEmpty()) {
             int i = wallCollisionList.get(0);
             RobotPlacement r = next.get(i);
             RobotPlacement rp = robots.get(i);
             Action act = actions.get(i);
             boolean noCollision = true;
             // VERTICALS
             for (int v = (int) Math.floor(r.x - ROBOT_RADIUS); v <= (int) Math.ceil(r.x + ROBOT_RADIUS); v++) {
                 if (v >= 0 && v <= this.SIZE_X) //check for array bounds //perhaps will be redundant after ST_ERR modifications
                     for (Line line : environmentMap.vertical[v]) {
                         if ((line.start <= r.y + ROBOT_RADIUS) && (line.end >= r.y - ROBOT_RADIUS)) {//Bounding box check
                             if (!resolvePossibleCollisions(i, r, rp, act, line))
                                noCollision = false;
                        }
                     }
             }
             // HORIZONTALS
             for (int h = (int) Math.floor(r.y - ROBOT_RADIUS); h <= (int) Math.ceil(r.y + ROBOT_RADIUS); h++) {
                 if (h >= 0 && h <= this.SIZE_Y) //check for array bounds //perhaps will be redundant after ST_ERR modifications
                     for (Line line : environmentMap.horizontal[h]) {
                         if ((line.start <= r.x + ROBOT_RADIUS) && (line.end >= r.x - ROBOT_RADIUS)) {//Bounding box check
                             if (!resolvePossibleCollisions(i, r, rp, act, line))
                                 noCollision = false;
                         }
                     }
             }
             if (noCollision) {
                 wallCollisionList.remove(0);
             }
         }
    }
        // --- Resolving robot-robot collisions:
        ArrayList<Pair<Integer, Integer>> listOfCollisions = new ArrayList<>();
        for (int i = 0; i < distances.length; i++) {
            for (int j = 0; j < distances[i].length; j++) {
                if (i > j) {
                    distances[i][j] = distance(next.get(i), next.get(j));
                    if (distances[i][j] < DOUBLE_RADIUS_SQUARED - ST_ERR) {
                        listOfCollisions.add(new Pair<>(i, j));
                    }
                }
            }
        }

        while (!listOfCollisions.isEmpty()) {
            Pair<Integer, Integer> p = listOfCollisions.remove(0);
            int i = p.getKey();
            int j = p.getValue();

            if (distances[i][j] < DOUBLE_RADIUS_SQUARED - 0.1) {
                if (actions.get(i).degreeOfRealization > actions.get(j).degreeOfRealization) {
                    resolveCollision(robots.get(i), next.get(i), actions.get(i), robots.get(j), next.get(j), actions.get(j));
                } else {
                    resolveCollision(robots.get(j), next.get(j), actions.get(j), robots.get(i), next.get(i), actions.get(i));
                }
                distances[i][j] = DOUBLE_RADIUS_SQUARED;

                for (int k = 0; k < distances[i].length; k++) {
                    if (i != k && j != k) {
                        distances[i][k] = distance(next.get(i), next.get(k));
                        if (distances[i][k] < DOUBLE_RADIUS_SQUARED - ST_ERR) {
                            listOfCollisions.add(new Pair<>(i, k));
                        }
                    }
                }
                for (int k = 0; k < distances[j].length; k++) {
                    if (i != k && j != k) {
                        distances[j][k] = distance(next.get(j), next.get(k));
                        if (distances[j][k] < DOUBLE_RADIUS_SQUARED - ST_ERR) {
                            listOfCollisions.add(new Pair<>(j, k));
                        }
                    }
                }
            }
        }
        robots = next;
        actualizedActions = false;
    }

    private boolean resolvePossibleCollisions(int robotNumber, RobotPlacement r, RobotPlacement rp, Action act, Line line) {
        //Computing distances:
        double lineDistance, startDistance, endDistance, coordinate1, coordinate2;

        if (line.isVertical) {
            coordinate1 = r.x;
            coordinate2 = r.y;
        } else {
            coordinate1 = r.y;
            coordinate2 = r.x;
        }
        if (line.start > coordinate2 || line.end < coordinate2) {
            lineDistance = Double.MAX_VALUE;
        } else {
            lineDistance = Math.abs(line.horizon - coordinate1);
        }
        startDistance = Math.pow(coordinate1 - line.horizon, 2) + Math.pow(coordinate2 - line.start, 2);
        endDistance = Math.pow(coordinate1 - line.horizon, 2) + Math.pow(coordinate2 - line.end, 2);

        if (lineDistance <= ROBOT_RADIUS || startDistance <= ROBOT_RADIUS_SQUARED || endDistance <= ROBOT_RADIUS_SQUARED) {
            double a = resolveWallCollision(r, rp, act, line, startDistance <= ROBOT_RADIUS_SQUARED, endDistance <= ROBOT_RADIUS_SQUARED);
            if (a == a) {
                collisionList.addWallCollision(robotNumber, r.x, r.y, a);
            }
        }
        return !(lineDistance < ROBOT_RADIUS - ST_ERR_SQR || startDistance < ROBOT_RADIUS_SQUARED - ST_ERR ||
                endDistance < ROBOT_RADIUS_SQUARED - ST_ERR);
    }

    public static boolean checkConsistency(RobotPlacement r, Line l) {
        double lineDistance, startDistance, endDistance, coordinate1, coordinate2;

        if (l.isVertical) {
            coordinate1 = r.x;
            coordinate2 = r.y;
        } else {
            coordinate1 = r.y;
            coordinate2 = r.x;
        }

        if (l.start > coordinate2 || l.end < coordinate2) {
            lineDistance = Double.MAX_VALUE;
        } else {
            lineDistance = Math.abs(l.horizon - coordinate1);
        }
        startDistance = Math.pow(coordinate1 - l.horizon, 2) + Math.pow(coordinate2 - l.start, 2);
        endDistance = Math.pow(coordinate1 - l.horizon, 2) + Math.pow(coordinate2 - l.end, 2);

        return !(lineDistance < ROBOT_RADIUS - ST_ERR_SQR || startDistance < ROBOT_RADIUS_SQUARED - ST_ERR ||
                endDistance < ROBOT_RADIUS_SQUARED - ST_ERR);
    }

    private double resolveWallCollision(RobotPlacement r, RobotPlacement rp, Action act, Line line, boolean startCollision, boolean endCollision) {
        //if there is no collision, just touch, we do not need to resolve anything, just compute an angle
        if (act.degreeOfRealization == 0 || act.velocity == 0) {
            if (startCollision) {
                return subjectiveAngleBetween(r, line.start, line.horizon);
            } else if (endCollision) {
                return subjectiveAngleBetween(r, line.end, line.horizon);
            } else {
                return subjectiveAngleBetween(r, r.x, line.horizon);
            }
        }

        double degree = act.degreeOfRealization;
        double tx = Double.NaN;
        double point = 0;
        double ts;
        if (startCollision) {
            if (line.isVertical) {
                tx = solveQuadraticEquation(rp, act, line.horizon, line.start, ROBOT_RADIUS_SQUARED, 0, degree);
            } else  {
                tx = solveQuadraticEquation(rp, act, line.start, line.horizon, ROBOT_RADIUS_SQUARED, 0, degree);
            }
            updateCoordinates(r, rp, act, tx);
            if (!checkConsistency(r, line)) {
                tx = Double.NaN;
            }
            point = line.start;
        }
        if (endCollision && tx != tx) {
            if (line.isVertical) {
                tx = solveQuadraticEquation(rp, act, line.horizon, line.end, ROBOT_RADIUS_SQUARED, 0, degree);
            } else {
                tx = solveQuadraticEquation(rp, act, line.end, line.horizon, ROBOT_RADIUS_SQUARED, 0, degree);
            }
            updateCoordinates(r, rp, act, tx);
            if (!checkConsistency(r, line)) {
                tx = Double.NaN;
            }
            point = line.end;
        }

        if (line.isVertical) {
            if (line.horizon < rp.x) {
                ts = bound((ROBOT_RADIUS - rp.x + line.horizon)/(act.velocity*Math.sin(rp.angle)), 0, degree);
            } else {
                ts = bound((line.horizon - rp.x - ROBOT_RADIUS)/(act.velocity*Math.sin(rp.angle)), 0, degree);
            }
        } else {
            if (line.horizon < rp.y) {
                ts = bound((ROBOT_RADIUS - rp.y + line.horizon)/(act.velocity*Math.cos(rp.angle)), 0, degree);
            } else {
                ts = bound((line.horizon - rp.y - ROBOT_RADIUS)/(act.velocity*Math.cos(rp.angle)), 0, degree);
            }
        }
        if ((tx != tx || tx < ts) && ts == ts) {
            updateCoordinates(r, rp, act, ts);
            if (line.isVertical) {
                return subjectiveAngleBetween(r, line.horizon, r.y);
            } else {
                return subjectiveAngleBetween(r, r.x, line.horizon);
            }

        } else if (tx == tx) {
            if (line.isVertical) {
                return subjectiveAngleBetween(r, line.horizon, point);
            } else {
                return subjectiveAngleBetween(r, point, line.horizon);
            }

        } else {
            updateCoordinates(r, rp, act, 0);
            return Double.NaN;
        }
    }

    /**
     * Resolves collision for a case where one robot is static and the other one moves
     */
    private void resolveCollision(RobotPlacement rp1, RobotPlacement r1, Action act1, RobotPlacement rp2, RobotPlacement r2, Action act2) {
        if (act1.degreeOfRealization != act2.degreeOfRealization) {
            double t = solveQuadraticEquation(rp1, act1, r2.x, r2.y, DOUBLE_RADIUS_SQUARED, act2.degreeOfRealization, 1);

            if (t == t) {
                updateCoordinates(r1, rp1, act1, t);
            } else {
                resolveCollision2(rp1, r1, act1, rp2, r2, act2);
            }
        } else {
            resolveCollision2(rp1, r1, act1, rp2, r2, act2);
        }
    }

    private void updateCoordinates(RobotPlacement r, RobotPlacement rp, Action act, double degree) {
        r.x = rp.x + act.velocity * Math.sin(rp.angle) * degree;
        r.y = rp.y + act.velocity * Math.cos(rp.angle) * degree;
        act.degreeOfRealization = degree;
    }

    //NOTE: bounds may be wrong
    /**
     * Resolves collision for a case where both robots move simultaneously.
     */
    private void resolveCollision2(RobotPlacement rp1, RobotPlacement r1, Action act1, RobotPlacement rp2, RobotPlacement r2, Action act2) {
        double A = rp1.x - rp2.x;
        double B = rp1.y - rp2.y;
        double N = act1.velocity * Math.sin(rp1.angle) - act2.velocity * Math.sin(rp2.angle);
        double M = act1.velocity * Math.cos(rp1.angle) - act2.velocity * Math.cos(rp2.angle);

        double a = N*N + M*M;
        double b = 2*A*N + 2*B*M;
        double c = A*A + B*B - DOUBLE_RADIUS_SQUARED;

        double det = b*b - 4*a*c;
        double t1 = bound(( - b + Math.sqrt(det)) / (2*a), 0, 1);
        double t2 = bound(( - b - Math.sqrt(det)) / (2*a), 0, 1);
        double t;
        if (t1 == t1) {
            t = t1;
        } else if (t2 == t2) {
            t = t2;
        } else {
            t = 0;
        }
        updateCoordinates(r1, rp1, act1, t);
        updateCoordinates(r2, rp2, act2, t);
    }

    private double solveQuadraticEquation(RobotPlacement rp, Action act, double x, double y, double distance, double lowerBound, double upperBound) {
        double A = rp.x - x;
        double B = rp.y - y;
        double N = act.velocity * Math.sin(rp.angle);
        double M = act.velocity * Math.cos(rp.angle);

        double a = N*N + M*M;
        double b = 2*A*N + 2*B*M;
        double c = A*A + B*B - distance;

        double det = b*b - 4*a*c;
        double t1 = bound(( - b + Math.sqrt(det)) / (2*a), lowerBound, upperBound);
        double t2 = bound(( - b - Math.sqrt(det)) / (2*a), lowerBound, upperBound);
        if (t1 == t1) {
            return t1;
        } else if (t2 == t2) {
            return t2;
        } else {
            return Double.NaN;
        }
    }

    private double bound(double t, double lowerBound, double upperBound) {
        if (t > lowerBound - ST_ERR_SQR && t <= lowerBound) {
            return lowerBound;
        } else if (t >= upperBound && t < upperBound + ST_ERR_SQR) {
            return upperBound;
        } else if (t > lowerBound && t < upperBound) {
            return t;
        } else {
            return Double.NaN;
        }
    }

    static double distance(RobotPlacement r1, RobotPlacement r2) {
        double xDist = (r1.x - r2.x);
        double yDist = (r1.y - r2.y);
        return xDist*xDist + yDist*yDist;
    }

    private static double angleBetween(double subjectX, double subjectY, double objectX, double objectY) {
        return Math.atan2(objectX - subjectX, objectY - subjectY);
    }

    static double subjectiveAngleBetween(RobotPlacement subject, RobotPlacement object) {
        return subjectiveAngleBetween(subject, object.x, object.y);
    }

    static double subjectiveAngleBetween(RobotPlacement subject, double objectX, double objectY) {
        return normalizeAngle(angleBetween(subject.x, subject.y, objectX, objectY) - subject.angle);
    }

    private boolean endCondition() {
        return step >= STEPS;
    }
}
