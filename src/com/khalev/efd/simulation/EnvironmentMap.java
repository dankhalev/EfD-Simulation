package com.khalev.efd.simulation;

import java.util.ArrayList;

//TODO: check unchecked assignments
//TODO: add getters instead of public fields
public class EnvironmentMap {

    public ArrayList<Line>[] vertical;
    public ArrayList<Line>[] horizontal;
    public int sizeX;
    public int sizeY;

    EnvironmentMap(int sizeX, int sizeY) {
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
