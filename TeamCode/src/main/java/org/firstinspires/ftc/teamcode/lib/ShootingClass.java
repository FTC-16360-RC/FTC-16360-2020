package org.firstinspires.ftc.teamcode.lib;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class ShootingClass {

    private Thread adjustingThread;
    private HashMap<String, Double[]> targets = new HashMap<String, Double[]>();
    private String alliance;
    private enum Targets  {
        blueUpper (1,2, 3),
        blueMiddle(1,2, 3),
        blueLower (1,2, 3),
        bluePsOuter (1,2, 3),
        bluePsMiddle (1,2, 3),
        bluePsInner (1,2, 3),
        redUpper (1,2, 3),
        redMiddle (1,2, 3),
        redLower (1,2, 3),
        redPsOuter (1,2, 3),
        redPsMiddle (1,2, 3),
        redPsInner (1,2, 3);

        private Targets(double x, double y, double z){

        }
    }
    private String currentTarget;


    private void ShootingClass(String alliance) {
        if (!(alliance == "Red")) {
            alliance = "blue";
            targets.put("blueUpper", new Double[]{1.0, 2.0, 3.0});
            targets.put("blueMiddle", new Double[]{1.0, 2.0, 3.0});
            targets.put("blueLower", new Double[]{1.0, 2.0, 3.0});
            targets.put("bluePsOuter", new Double[]{1.0, 2.0, 3.0});
            targets.put("bluePsMiddle", new Double[]{1.0, 2.0, 3.0});
            targets.put("bluePsInner", new Double[]{1.0, 2.0, 3.0});
        }
        if (!(alliance == "Blue")) {
            alliance = "red";
            targets.put("redUpper", new Double[] {1.0,2.0,3.0});
            targets.put("redMiddle", new Double[] {1.0,2.0,3.0});
            targets.put("redLower", new Double[] {1.0,2.0,3.0});
            targets.put("redPsOuter", new Double[] {1.0,2.0,3.0});
            targets.put("redPsMiddle", new Double[] {1.0,2.0,3.0});
            targets.put("redPsInner", new Double[] {1.0,2.0,3.0});
        }
    }

    private double calculateDistance() {

    }
    private double calculateAngleAlpha() {

    }
    private double calculateAngleTheta() {

    }
    private boolean motorsReady() {
        double pos1 = ShooterMotor1.getPosition() + ShooterMotor1.getPosition();
        Thread.sleep(5);
        double pos2 = ShooterMotor1.getPosition() + ShooterMotor1.getPosition();

        if (pos2 - pos1 > 15) {  //TUNE / CALCULATE / WHATEVER 15.
            return true;
        }
        return false;
    }
    public void adjustAngle() {
    double alpha = calculateAngleAlpha();
    }
    private void shootOnce(){

    }
    public void shootAll() {
        adjustAngle();
        // switch to Goal-Centric
        for (int i = 0; i < 3; i++) {
            while (!motorsReady()) {}
            adjustAngle();
            shootOnce();
        }
        // switch to Robot-Centric
    }
}
