package com.khalev;

import cz.cuni.mff.d3s.deeco.runtime.RuntimeFramework;
import javafx.util.Pair;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

/**
 * This class is used for computing cycles of the simulation and for writing logs.
 */
public class Environment {

    public static final double ROBOT_RADIUS = 3;
    public static final double DOUBLE_RADIUS_SQUARED = (2 * ROBOT_RADIUS) * (2 * ROBOT_RADIUS);
    public static final int CYCLE = 10;
    private static final double ST_ERR = 0.1;

    public final int SIZE_X;
    public final int SIZE_Y;
    public final int STEPS;

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
        this.runtime = runtime;
    }

    private ArrayList<RobotPlacement> robots;
    private double[][] distances;

    public ArrayList<Action> actions = new ArrayList<>();
    public void updateActions(Action[] acts) {
        actions.clear();
        if (acts.length == robots.size()) {
            for (Action a :
                    acts) {
                actions.add(a);
            }
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

    public boolean actualizedActions = false;
    public boolean actualizedSpatialInputs = false;


    public Environment(int sizeX, int sizeY, int numOfSteps, ArrayList<RobotPlacement> robots, File file) {
        SIZE_X = sizeX;
        SIZE_Y = sizeY;
        STEPS = numOfSteps;
        try {
            this.robots = robots;
            this.distances = new double[robots.size()][robots.size()];
            logfile = new FileWriter(file);
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
        System.out.println("CYCLE " + step);
        if (!endCondition()) {
            performActions();
            sendInputs();
            step++;
            return 0;
        } else {
            try {
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
        for (int i = 0; i < robots.size(); i++) {
            RobotPlacement r = robots.get(i);

            if (r.x == ROBOT_RADIUS) {
                double angle = normalizeAngle(Math.atan2( -3, 0) - r.angle);
                inputs.get(i).collisionPoints.add(angle);
            }
            if (r.x == SIZE_X - ROBOT_RADIUS) {
                double angle = normalizeAngle(Math.atan2( 3, 0) - r.angle);
                inputs.get(i).collisionPoints.add(angle);
            }
            if (r.y == ROBOT_RADIUS) {
                double angle = normalizeAngle(Math.atan2( 0, -3) - r.angle);
                inputs.get(i).collisionPoints.add(angle);
            }
            if (r.y == SIZE_Y - ROBOT_RADIUS) {
                double angle = normalizeAngle(Math.atan2( 0, 3) - r.angle);
                inputs.get(i).collisionPoints.add(angle);
            }
        }
        ///////////////////////////////

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
                    double angle1 = normalizeAngle(Math.atan2(r2.x - r1.x, r2.y - r1.y) - r1.angle);
                    double angle2 = normalizeAngle(Math.atan2(r1.x - r2.x, r1.y - r2.y) - r2.angle);
                    inputs.get(i).collisionPoints.add(angle1);
                    inputs.get(j).collisionPoints.add(angle2);
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

    /**
     * Resolves collision for a case where one robot is static and the other one moves
     */
    private void resolveCollision(RobotPlacement rp1, RobotPlacement r1, Action act1, RobotPlacement rp2, RobotPlacement r2, Action act2) {
        if (act1.degreeOfRealization != act2.degreeOfRealization) {
            double A = rp1.x - r2.x;
            double B = rp1.y - r2.y;
            double N = act1.velocity * Math.sin(rp1.angle);
            double M = act1.velocity * Math.cos(rp1.angle);

            double a = N*N + M*M;
            double b = 2*A*N + 2*B*M;
            double c = A*A + B*B - DOUBLE_RADIUS_SQUARED;

            double det = b*b - 4*a*c;
            double t1 = ( - b + Math.sqrt(det)) / (2*a);
            double t2 = ( - b - Math.sqrt(det)) / (2*a);

            if (t1 <= 1 && t1 >= act2.degreeOfRealization) {
                r1.x = rp1.x + act1.velocity * Math.sin(rp1.angle) * t1;
                r1.y = rp1.y + act1.velocity * Math.cos(rp1.angle) * t1;
                act1.degreeOfRealization = t1;
            } else if (t2 <= 1 && t2 >= act2.degreeOfRealization - ST_ERR) {
                r1.x = rp1.x + act1.velocity * Math.sin(rp1.angle) * t2;
                r1.y = rp1.y + act1.velocity * Math.cos(rp1.angle) * t2;
                act1.degreeOfRealization = t2;
            } else {
                resolveCollision2(rp1, r1, act1, rp2, r2, act2);
            }
        } else {
            resolveCollision2(rp1, r1, act1, rp2, r2, act2);
        }
    }

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
        double t1 = ( - b + Math.sqrt(det)) / (2*a);
        double t2 = ( - b - Math.sqrt(det)) / (2*a);
        double t;
        if (t1 <= 1 && t1 >= 0) {
            t = t1;
        } else if (t2 <= 1 && t2 >= 0) {
            t = t2;
        } else {
            if (Math.abs(0.5 - t1) < Math.abs(0.5 - t2)) {
                t = t1;
            } else if (Math.abs(0.5 - t1) > Math.abs(0.5 - t2)) {
                t = t2;
            } else {
                t = 0;
            }
        }
        r1.x = rp1.x + act1.velocity * Math.sin(rp1.angle) * t;
        r1.y = rp1.y + act1.velocity * Math.cos(rp1.angle) * t;
        act1.degreeOfRealization = t;
        r2.x = rp2.x + act2.velocity * Math.sin(rp2.angle) * t;
        r2.y = rp2.y + act2.velocity * Math.cos(rp2.angle) * t;
        act2.degreeOfRealization = t;
    }

    static double distance(RobotPlacement r1, RobotPlacement r2) {
        double xDist = (r1.x - r2.x);
        double yDist = (r1.y - r2.y);
        return xDist*xDist + yDist*yDist;
    }

    //TO-DO: add another conditions
    private boolean endCondition() {
        return step >= STEPS;
    }
}
