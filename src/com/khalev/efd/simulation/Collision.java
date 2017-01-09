package com.khalev.efd.simulation;

class Collision {

    Type type;
    int num1;
    int num2;

    Collision(Type type, int num1, int num2) {
        this.type = type;
        this.num1 = num1;
        this.num2 = num2;
    }

    enum Type {
        NONE, WALL, ROBOT
    }
}
