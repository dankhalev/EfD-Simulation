package com.khalev.efd.simulation;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

public class BitmapProcessor {

    BufferedImage img;
    private int sizeX;
    private int sizeY;

    public BitmapProcessor(File bitmap) throws IOException {
        this.img = ImageIO.read(bitmap);
        this.sizeX = img.getWidth();
        this.sizeY = img.getHeight();
    }

    public EnvironmentMap readBitmap() {
        EnvironmentMap map = new EnvironmentMap(sizeX, sizeY);

        for (int y = -1; y <= sizeY; y++) {
            for (int x = -1; x <= sizeX; x++) {
                if (isBlack(x, y)) {
                    if (!isBlack(x, y-1))
                        map.horizontal[y].add(new Line(x, x+1, y, false));
                    if (!isBlack(x, y+1))
                        map.horizontal[y+1].add(new Line(x, x+1, y+1, false));
                    if (!isBlack(x-1, y))
                        map.vertical[x].add(new Line(y, y+1, x, true));
                    if (!isBlack(x+1, y))
                        map.vertical[x+1].add(new Line(y, y+1, x+1, true));
                }
            }
        }

        for (int i = 0; i < map.horizontal.length; i++) {
            for (int j = 0; j < map.horizontal[i].size(); j++) {
                boolean b = true;
                while (b) {
                    Line start = map.horizontal[i].get(j);
                    if (j == map.horizontal[i].size() - 1) {
                        b = false;
                    } else {
                        Line next = map.horizontal[i].get(j + 1);
                        if (start.end == next.start) {
                            map.horizontal[i].remove(j);
                            map.horizontal[i].remove(j);
                            map.horizontal[i].add(j, new Line(start.start, next.end, start.horizon, start.isVertical));
                        } else {
                            b = false;
                        }
                    }
                }
            }
        }

        for (int i = 0; i < map.vertical.length; i++) {
            for (int j = 0; j < map.vertical[i].size(); j++) {
                boolean b = true;
                while (b) {
                    Line start = map.vertical[i].get(j);
                    if (j == map.vertical[i].size() - 1) {
                        b = false;
                    } else {
                        Line next = map.vertical[i].get(j + 1);
                        if (start.end == next.start) {
                            map.vertical[i].remove(j);
                            map.vertical[i].remove(j);
                            map.vertical[i].add(j, new Line(start.start, next.end, start.horizon, start.isVertical));
                        } else {
                            b = false;
                        }
                    }
                }
            }
        }
        return map;
    }

    private boolean isBlack(int x, int y) {
        if (x < 0 || y < 0 || x >= sizeX || y >= sizeY) {
            return true;
        }
        int rgb = img.getRGB(x, y);
        int red = (rgb >> 16) & 0x000000FF;
        int green = (rgb >> 8) & 0x000000FF;
        int blue = (rgb) & 0x000000FF;
        return red == 0 && green == 0 && blue == 0;
    }
}
