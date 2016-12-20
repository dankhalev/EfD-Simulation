package com.khalev.efd.simulation;

import javafx.util.Pair;
import java.util.ArrayList;

import static java.lang.Double.NaN;

//TODO: review ST_ERR method
//TODO: make a proof that computations work correctly
class SimulationEngine extends SensoryInputsProcessor {

    private ArrayList<RobotPlacement> robots;
    private EnvironmentMap environmentMap;
    private CollisionList collisionList;
    private ArrayList<Action> actions;

    private static final double ST_ERR = 0.01;
    private static final double ROBOT_RADIUS = 3;
    private static final double ROBOT_RADIUS_SQUARED = ROBOT_RADIUS * ROBOT_RADIUS;
    private static final double DOUBLE_RADIUS_SQUARED = (2 * ROBOT_RADIUS) * (2 * ROBOT_RADIUS);


    public SimulationEngine(ArrayList<RobotPlacement> robots, EnvironmentMap map) {
        super(map);
        this.environmentMap = map;
        this.collisionList = new CollisionList(robots.size());
        this.robots = robots;
    }

    ArrayList<RobotPlacement> performActions(ArrayList<Action> actions) {
        this.actions = actions;
        // --- Creating a placement list for next step:
        ArrayList<RobotPlacement> next = new ArrayList<>();
        for (RobotPlacement rp : robots) {
            next.add(rp.copy());
        }

        // --- Performing actions:
        for (int i = 0; i < actions.size(); i++) {
            Action act = actions.get(i);
            RobotPlacement r = next.get(i);

            switch (act.type) {
                case ROTATE:
                    r.angle = normalizeAngle(r.angle + act.angle);
                    act.degreeOfRealization = 0;
                    break;
                case STAY:
                    act.degreeOfRealization = 0;
                    break;
                case MOVE:
                    r.x += act.velocity * Math.sin(r.angle);
                    r.y += act.velocity * Math.cos(r.angle);
                    act.degreeOfRealization = 1;
                    break;
            }
        }

        // --- Resolving robot-wall collisions:
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

            int lowerVerticalBound = (int) Math.max(Math.ceil(r.x - ROBOT_RADIUS - ST_ERR), 0);
            int upperVerticalBound = (int) Math.min(Math.floor(r.x + ROBOT_RADIUS + ST_ERR), environmentMap.sizeX);
            int lowerHorizontalBound = (int) Math.max(Math.ceil(r.y - ROBOT_RADIUS - ST_ERR), 0);
            int upperHorizontalBound = (int) Math.min(Math.floor(r.y + ROBOT_RADIUS + ST_ERR), environmentMap.sizeY);

            for (int v = lowerVerticalBound; v <= upperVerticalBound; v++) {
                for (Line line : environmentMap.vertical[v]) {
                    if ((line.start <= r.y + ROBOT_RADIUS) && (line.end >= r.y - ROBOT_RADIUS)) {//Bounding box check
                        if (!resolvePossibleCollisions(i, r, rp, act, line))
                            noCollision = false;
                    }
                }
            }
            for (int h = lowerHorizontalBound; h <= upperHorizontalBound; h++) {
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

        // --- Resolving robot-robot collisions:

        ArrayList<Pair<Integer, Integer>> listOfCollisions = new ArrayList<>();
        for (int i = 0; i < robots.size(); i++) {
            for (int j = 0; j < robots.size(); j++) {
                if (i > j) {
                    if (distance(next.get(i), next.get(j)) < DOUBLE_RADIUS_SQUARED - ST_ERR) {
                        listOfCollisions.add(new Pair<>(i, j));
                    }
                }
            }
        }

        while (!listOfCollisions.isEmpty()) {
            Pair<Integer, Integer> p = listOfCollisions.remove(0);
            int i = p.getKey();
            int j = p.getValue();

            if (distance(next.get(i), next.get(j)) < DOUBLE_RADIUS_SQUARED - ST_ERR) {
                if (actions.get(i).degreeOfRealization > actions.get(j).degreeOfRealization) {
                    resolveCollisionForOneMovingBody(robots.get(i), next.get(i), actions.get(i), robots.get(j), next.get(j), actions.get(j));
                } else {
                    resolveCollisionForOneMovingBody(robots.get(j), next.get(j), actions.get(j), robots.get(i), next.get(i), actions.get(i));
                }

                for (int k = 0; k < robots.size(); k++) {
                    if (i != k && j != k) {
                        if (distance(next.get(i), next.get(k)) < DOUBLE_RADIUS_SQUARED - ST_ERR) {
                            listOfCollisions.add(new Pair<>(i, k));
                        }
                    }
                }
                for (int k = 0; k < robots.size(); k++) {
                    if (i != k && j != k) {
                        if (distance(next.get(j), next.get(k)) < DOUBLE_RADIUS_SQUARED - ST_ERR) {
                            listOfCollisions.add(new Pair<>(j, k));
                        }
                    }
                }
            }
        }
        this.robots = next;

        return this.robots;
    }

    ArrayList<SpatialInput> sendInputs(ArrayList<RobotPlacement> robots) {
        assert actions.size() == robots.size(): "Actions were not updated";
        ArrayList<SpatialInput> inputs = new ArrayList<>();

        for (int i = 0; i < robots.size(); i++) {
            inputs.add(new SpatialInput(actions.get(i)));
        }

        collisionList.computeCollisionsWithWalls(robots, inputs);

        for (int i = 0; i < robots.size(); i++) {
            for (int j = 0; j < robots.size(); j++) {
                if (i > j && distance(robots.get(i), robots.get(j)) < DOUBLE_RADIUS_SQUARED + ST_ERR) {
                    RobotPlacement r1 = robots.get(i);
                    RobotPlacement r2 = robots.get(j);
                    inputs.get(i).collisionPoints.add(subjectiveAngleBetween(r1, r2));
                    inputs.get(j).collisionPoints.add(subjectiveAngleBetween(r2, r1));
                }
            }
        }

        return inputs;
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
                collisionList.addWallCollision(robotNumber, r.x, r.y, r.angle, a);
            }
        }
        return !(lineDistance < ROBOT_RADIUS - ST_ERR || startDistance < ROBOT_RADIUS_SQUARED - ST_ERR ||
                endDistance < ROBOT_RADIUS_SQUARED - ST_ERR);
    }

    /**
     * Resolves collision between robot and wall, updates robot's position
     * @param r
     * @param rp
     * @param act
     * @param line
     * @param startCollision
     * @param endCollision
     * @return subjective angle of collision between robot and wall
     */
    private double resolveWallCollision(RobotPlacement r, RobotPlacement rp, Action act, Line line, boolean startCollision, boolean endCollision) {
        //if there is no collision, just touch, we do not need to resolve anything, just compute an angle
        if (act.degreeOfRealization == 0 || act.velocity == 0) {
            assert !startCollision || !endCollision: "Two collision points for static wall collision";
            if (startCollision) {
                if (line.isVertical) {
                    return subjectiveAngleBetween(r, line.horizon, line.start);
                }
                return subjectiveAngleBetween(r, line.start, line.horizon);
            } else if (endCollision) {
                if (line.isVertical) {
                    return subjectiveAngleBetween(r, line.horizon, line.end);
                }
                return subjectiveAngleBetween(r, line.end, line.horizon);
            } else {
                if (line.isVertical) {
                    return subjectiveAngleBetween(r, line.horizon, r.y);
                }
                return subjectiveAngleBetween(r, r.x, line.horizon);
            }
        }

        double degree = act.degreeOfRealization;
        double tx = NaN;
        double point = 0;
        double ts = NaN;
        //resolving edge collisions:
        if (startCollision) {
            if (line.isVertical) {
                tx = solveQuadraticEquation(rp, act, line.horizon, line.start, ROBOT_RADIUS_SQUARED, 0, degree);
            } else  {
                tx = solveQuadraticEquation(rp, act, line.start, line.horizon, ROBOT_RADIUS_SQUARED, 0, degree);
            }
            updateCoordinates(r, rp, act, tx);
            //if position is still inconsistent, we can't use this solution:
            if (!checkConsistency(r, line)) {
                tx = NaN;
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
                tx = NaN;
            }
            point = line.end;
        }
        //resolving line collision:
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
        assert tx == tx || ts == ts: "No solution was found for robot-wall collision";
        assert (tx != tx || ts != ts) || tx > ts: "Impossible state occurred: tx < ts, tx = " + tx + ", ts = " + ts;

        if (tx == tx) {
            if (line.isVertical) {
                return subjectiveAngleBetween(r, line.horizon, point);
            } else {
                return subjectiveAngleBetween(r, point, line.horizon);
            }
        } else {
            updateCoordinates(r, rp, act, ts);
            if (line.isVertical) {
                return subjectiveAngleBetween(r, line.horizon, r.y);
            } else {
                return subjectiveAngleBetween(r, r.x, line.horizon);
            }
        }
    }

    private void resolveCollisionForOneMovingBody(RobotPlacement rp1, RobotPlacement r1, Action act1, RobotPlacement rp2, RobotPlacement r2, Action act2) {
        assert distance(r1, r2) < DOUBLE_RADIUS_SQUARED - ST_ERR: "Illegal resolveCollisionForOneMovingBody call";
        if (act1.degreeOfRealization != act2.degreeOfRealization) {
            double t = solveQuadraticEquation(rp1, act1, r2.x, r2.y, DOUBLE_RADIUS_SQUARED, act2.degreeOfRealization, act1.degreeOfRealization);

            if (t == t) {
                updateCoordinates(r1, rp1, act1, t);
            } else {
                resolveCollisionForTwoMovingBodies(rp1, r1, act1, rp2, r2, act2);
            }
        } else {
            resolveCollisionForTwoMovingBodies(rp1, r1, act1, rp2, r2, act2);
        }
    }

    private void updateCoordinates(RobotPlacement r, RobotPlacement rp, Action act, double degree) {
        assert degree >= 0 && degree <= 1 && degree <= act.degreeOfRealization: "There was an attempt to update action " +
                "with greater value: " + degree + " instead of " + act.degreeOfRealization;
        r.x = rp.x + act.velocity * Math.sin(rp.angle) * degree;
        r.y = rp.y + act.velocity * Math.cos(rp.angle) * degree;
        act.degreeOfRealization = degree;
    }

    private void resolveCollisionForTwoMovingBodies(RobotPlacement rp1, RobotPlacement r1, Action act1, RobotPlacement rp2, RobotPlacement r2, Action act2) {
        double bound = Math.min(act1.degreeOfRealization, act2.degreeOfRealization);
        double A = rp1.x - rp2.x;
        double B = rp1.y - rp2.y;
        double N = act1.velocity * Math.sin(rp1.angle) - act2.velocity * Math.sin(rp2.angle);
        double M = act1.velocity * Math.cos(rp1.angle) - act2.velocity * Math.cos(rp2.angle);

        double a = N*N + M*M;
        double b = 2*A*N + 2*B*M;
        double c = A*A + B*B - DOUBLE_RADIUS_SQUARED;

        double det = b*b - 4*a*c;
        double t1 = bound(( - b + Math.sqrt(det)) / (2*a), 0, bound);
        double t2 = bound(( - b - Math.sqrt(det)) / (2*a), 0, bound);
        assert t1 == t1 || t2 == t2: "Quadratic equation for 2 moving bodies could not be solved correctly with values: " +
                t1 + " and " + t2;
        double t = 0;
        if(t2 == t2 && t1 == t1) {
            t = Math.max(t1, t2);
        } else if (t1 == t1) {
            t = t1;
        } else if (t2 == t2) {
            t = t2;
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
            return NaN;
        }
    }

    private double bound(double t, double lowerBound, double upperBound) {
        if (t > lowerBound - ST_ERR && t <= lowerBound) {
            return lowerBound;
        } else if (t >= upperBound && t < upperBound + ST_ERR) {
            return upperBound;
        } else if (t > lowerBound && t < upperBound) {
            return t;
        } else {
            return NaN;
        }
    }

    private static boolean checkConsistency(RobotPlacement r, Line l) {
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

        return !(lineDistance < ROBOT_RADIUS - ST_ERR || startDistance < ROBOT_RADIUS_SQUARED - ST_ERR ||
                endDistance < ROBOT_RADIUS_SQUARED - ST_ERR);
    }

    static Collision checkMapConsistency(ArrayList<RobotPlacement> robots, EnvironmentMap map) {
        //Checking for collisions with walls
        for (int i = 0; i < robots.size(); i++) {
            RobotPlacement r = robots.get(i);
            int lowerVerticalBound = (int) Math.max(Math.floor(r.x - ROBOT_RADIUS), 0);
            int upperVerticalBound = (int) Math.min(Math.ceil(r.x + ROBOT_RADIUS), map.sizeX);
            int lowerHorizontalBound = (int) Math.max(Math.floor(r.y - ROBOT_RADIUS), 0);
            int upperHorizontalBound = (int) Math.min(Math.ceil(r.y + ROBOT_RADIUS), map.sizeY);
            for (int v = lowerVerticalBound; v <= upperVerticalBound; v++) {
                for (Line line : map.vertical[v]) {
                    if (!checkConsistency(r, line)) {
                        return new Collision(Collision.Type.WALL, i, -1);
                    }
                }
            }
            for (int h = lowerHorizontalBound; h <= upperHorizontalBound; h++) {
                for (Line line : map.horizontal[h]) {
                    if (!checkConsistency(r, line)) {
                        return new Collision(Collision.Type.WALL, i, -1);
                    }
                }
            }
        }
        //Checking for collisions with robots
        for (int i = 0; i < robots.size(); i++) {
            for (int j = 0; j < robots.size(); j++) {
                if (i != j && distance(robots.get(i), robots.get(j)) < DOUBLE_RADIUS_SQUARED)
                    return new Collision(Collision.Type.ROBOT, i, j);
            }
        }

        return new Collision(Collision.Type.NONE, -1, -1);
    }

    static double normalizeAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    private static double angleBetween(double subjectX, double subjectY, double objectX, double objectY) {
        return Math.atan2(objectX - subjectX, objectY - subjectY);
    }

    private static double subjectiveAngleBetween(RobotPlacement subject, RobotPlacement object) {
        return subjectiveAngleBetween(subject, object.x, object.y);
    }

    private static double subjectiveAngleBetween(RobotPlacement subject, double objectX, double objectY) {
        return normalizeAngle(angleBetween(subject.x, subject.y, objectX, objectY) - subject.angle);
    }

    private static double distance(RobotPlacement r1, RobotPlacement r2) {
        double xDist = (r1.x - r2.x);
        double yDist = (r1.y - r2.y);
        return xDist*xDist + yDist*yDist;
    }
}
