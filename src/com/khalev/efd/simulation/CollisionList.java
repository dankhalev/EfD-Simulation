package com.khalev.efd.simulation;

import java.util.ArrayList;

public class CollisionList {

    private ArrayList<PotentialCollision>[] potentialCollisionList;

    public CollisionList(int size) {
        potentialCollisionList = new ArrayList[size];
        for (int i = 0; i < size; i++) {
            potentialCollisionList[i] = new ArrayList<>();
        }


    }

    public boolean addWallCollision(int robotNumber, double x, double y, double robotAngle, double angle){
        for (PotentialCollision pc : potentialCollisionList[robotNumber]) {
            if (pc.subjectiveCollisionAngle == angle && pc.originalX == x && pc.originalY == y) {
                return false;
            }
        }
        potentialCollisionList[robotNumber].add(new PotentialCollision(angle, x, y, robotAngle));
        return true;
    }

    public void computeCollisionsWithWalls(ArrayList<RobotPlacement> robots, ArrayList<SpatialInput> inputs) {
        for (int i = 0; i < potentialCollisionList.length; i++) {
            for (int j = potentialCollisionList[i].size() - 1; j >= 0; j--) {
                PotentialCollision coll = potentialCollisionList[i].get(j);
                RobotPlacement r = robots.get(i);
                if (r.x.equals(coll.originalX) && r.y.equals(coll.originalY) && r.angle.equals(coll.originalAngle)) {
                    inputs.get(i).collisionPoints.add(coll.subjectiveCollisionAngle);
                } else {
                    potentialCollisionList[i].remove(j);
                }
            }
        }
    }

    private class PotentialCollision {
        double subjectiveCollisionAngle;
        double originalAngle;
        double originalX;
        double originalY;

        public PotentialCollision(double subjectiveCollisionAngle, double originalX, double originalY, double originalAngle) {
            this.subjectiveCollisionAngle = subjectiveCollisionAngle;
            this.originalX = originalX;
            this.originalY = originalY;
            this.originalAngle = originalAngle;
        }
    }
}

