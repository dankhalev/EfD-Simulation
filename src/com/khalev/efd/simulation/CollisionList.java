package com.khalev.efd.simulation;

import java.util.ArrayList;

class CollisionList {

    private ArrayList<PotentialCollision>[] potentialCollisionList;

    CollisionList(int size) {
        @SuppressWarnings("unchecked")
        ArrayList<PotentialCollision>[] list = new ArrayList[size];
        potentialCollisionList = list;
        for (int i = 0; i < size; i++) {
            potentialCollisionList[i] = new ArrayList<>();
        }


    }

    boolean addWallCollision(int robotNumber, double x, double y, double robotAngle, double angle){
        for (PotentialCollision pc : potentialCollisionList[robotNumber]) {
            if (pc.subjectiveCollisionAngle == angle && pc.originalX == x && pc.originalY == y) {
                return false;
            }
        }
        potentialCollisionList[robotNumber].add(new PotentialCollision(angle, x, y, robotAngle));
        return true;
    }

    void computeCollisionsWithWalls(ArrayList<RobotPlacement> robots, ArrayList<CollisionData> inputs) {
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

        PotentialCollision(double subjectiveCollisionAngle, double originalX, double originalY, double originalAngle) {
            this.subjectiveCollisionAngle = subjectiveCollisionAngle;
            this.originalX = originalX;
            this.originalY = originalY;
            this.originalAngle = originalAngle;
        }
    }
}

