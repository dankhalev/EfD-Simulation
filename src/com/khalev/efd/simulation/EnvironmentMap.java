package com.khalev.efd.simulation;

import java.util.ArrayList;

public class EnvironmentMap {

    ArrayList<Line>[] vertical;
    ArrayList<Line>[] horizontal;
    int sizeX;
    int sizeY;

    public EnvironmentMap(int sizeX, int sizeY) {
        this.sizeX = sizeX;
        this.sizeY = sizeY;
        vertical = new ArrayList[sizeX+1];
        for (int i = 0; i <= sizeX; i++) {
            vertical[i] = new ArrayList();
        }
        horizontal = new ArrayList[sizeY+1];
        for (int i = 0; i <= sizeY; i++) {
            horizontal[i] = new ArrayList();
        }

    }
}
