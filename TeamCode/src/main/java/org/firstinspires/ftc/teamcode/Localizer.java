package org.firstinspires.ftc.teamcode;

import java.util.Vector;

public class Localizer {
    public double[] position = {0, 0, 0};// x, y, heading

    private int quadrant(){//returns the quadrant of the robot in the arena. We've divided the Into the Deep Arena into 8 quadrants
        int actualQuadrant = 0;
        if (position[0] <= 180 && !(position[1]>120 && position[1]<240)){
            if(position[1] <= 120){
                actualQuadrant = 1;
            }else if(position[1] > 240){
                actualQuadrant = 6;
            }
        } else if (position[0] > 180 && !(position[1]>120 && position[1]<240)) {
            if(position[1] <= 120){
                actualQuadrant = 2;
            }else if(position[1] > 240){
                actualQuadrant = 5;
            }
        }else {
            if (position[0] <= 120){
                actualQuadrant = 7;
            }else if (position[0] <= 147){
                actualQuadrant = 8;
            } else if (position[0] <= 267) {
               actualQuadrant = 4;
            }else {
                actualQuadrant = 3;
            }
        }
        return actualQuadrant;
    }

}
