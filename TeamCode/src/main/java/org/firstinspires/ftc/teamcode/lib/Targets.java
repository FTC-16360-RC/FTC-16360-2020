package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.arcrobotics.ftclib.geometry.Vector2d;

public class Targets {

    public static int currentTargetNum = 0;
    public static Vector2d currentTarget = new Vector2d(35, -72);
    public static String currentTargetName;

    //blue targets
    public static final Vector2d blueGoal = new Vector2d(72, 35);
    public static final Vector2d leftBluePowerShot = new Vector2d(72, 21);
    public static final Vector2d middleBluePowerShot = new Vector2d(72, 13.5);
    public static final Vector2d rightBluePowerShot = new Vector2d(72, 6);

    //red targets
    public static final Vector2d redGoal = new Vector2d(72, -35);
    public static final Vector2d leftRedPowerShot = new Vector2d(72, -6);
    public static final Vector2d middleRedPowerShot = new Vector2d(72, -13.5);
    public static final Vector2d rightRedPowerShot = new Vector2d(72, -21);

    public static void nextTarget() {
        currentTargetNum++;
        currentTargetNum = currentTargetNum % 4;
        getCoords();
    }

    public static void previousTarget() {
        currentTargetNum += 3;
        currentTargetNum = currentTargetNum % 4;
        getCoords();
    }

    static void getCoords() {
        switch (Comms.team) {
            case RED:
                switch (currentTargetNum) {
                    case 0:
                        currentTargetName = "redGoal";
                        currentTarget = redGoal;
                    case 1:
                        currentTargetName = "redLeftPowerShot";
                        currentTarget = leftRedPowerShot;
                    case 2:
                        currentTargetName = "redMiddletPowerShot";
                        currentTarget = middleRedPowerShot;
                    case 3:
                        currentTargetName = "redRightPowerShot";
                        currentTarget = rightRedPowerShot;
                }
                break;
            case BLU:
                switch (currentTargetNum) {
                    case 0:
                        currentTarget = blueGoal;
                        currentTargetName = "blbueGoal";
                    case 1:
                        currentTarget = leftBluePowerShot;
                        currentTargetName = "blueLeftPowerShot";
                    case 2:
                        currentTarget = middleBluePowerShot;
                        currentTargetName = "blueMiddlePowerShot";
                    case 3:
                        currentTarget = rightBluePowerShot;
                        currentTargetName = "blueRightPowerShot";
                }
                break;
        }
    }
}