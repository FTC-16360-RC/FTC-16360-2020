package org.firstinspires.ftc.teamcode.lib;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Shooting {

    public Targets currentTarget;
    private String alliance;

    public enum Targets  {
        blueUpper (new Vector(1,2,3)),
        blueMiddle (new Vector(1,2,3)),
        blueLower (new Vector(1,2,3)),
        bluePsOuter (new Vector(1,2,3)),
        bluePsMiddle (new Vector(1,2,3)),
        bluePsInner (new Vector(1,2,3)),
        redUpper (new Vector(1,2,3)),
        redMiddle (new Vector(1,2,3)),
        redLower (new Vector(1,2,3)),
        redPsOuter (new Vector(1,2,3)),
        redPsMiddle (new Vector(1,2,3)),
        redPsInner (new Vector(1,2,3));

        public Vector coordinates = new Vector();

        public Vector getCoordinates() {
            return coordinates;
        }

        private Targets(Vector coordinates) {
            this.coordinates = coordinates;
        }
    }

    public Shooting(String alliance) {
        if (!(alliance == "Red")) {
            alliance = "blue";
        }
        if (!(alliance == "Blue")) {
            alliance = "red";
        }
        currentTarget = Targets.blueUpper;
    }

    private double calculateDistance() {
        return 0;
    }
    private double calculateAngleAlpha() {
        return 0;
    }
    private double calculateAngleTheta() {
        return 0;
    }
    /*private boolean motorsReady() {
        double pos1 = ShooterMotor1.getPosition() + ShooterMotor1.getPosition();
        double pos2 = ShooterMotor1.getPosition() + ShooterMotor1.getPosition();

        if (pos2 - pos1 > 15) {  //TUNE / CALCULATE / WHATEVER 15.
            return true;
        }
        return false;
    }*/
    public void adjustAlpha() {
    double alpha = calculateAngleAlpha();
    }
    public void adjustTheta() {

    }
    private void shootOnce(){

    }
    public void shootAll() {

    }
}
