package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Globals {

    public enum Alliance {
        RED,
        BLUE
    }

    public static Alliance alliance = Alliance.BLUE;

    public static Targets.TargetType currentTargetType;

    public static Vector2d currentTarget = new Vector2d();

    public static AutoAim.Mode currentAimingMode = AutoAim.Mode.ALIGN_TO_HEADING;

    public static void setTarget(Targets.TargetType targetType) {
        currentTargetType = targetType;
    }

    public static void updateTarget() {
        int targetYArrayIndex = 0;
        switch (currentTargetType) {
            case HIGHGOAL:
                targetYArrayIndex = 0;
                break;
            case OUTER_POWERSHOT:
                targetYArrayIndex = 1;
                break;
            case CENTER_POWERSHOT:
                targetYArrayIndex = 2;
                break;
            case INNER_POWERSHOT:
                targetYArrayIndex = 3;
                break;
        }
        int mirrorCoefficient = 1;
        if(alliance == Alliance.RED)
            mirrorCoefficient = -1;
        currentTarget = new Vector2d(Targets.targetX, mirrorCoefficient*Targets.targetsY[targetYArrayIndex]);
    }

    public static double highGoalRPM = 5000;

    public static double powerShotRPM = 3500;

    public static double rpmToTicksPerSecond(double rpm, double gearRatio) {
        return rpm * 28 / gearRatio / 60;
    }

    public static double ticksPerSecondToRpm(double tps, double gearRatio) {
        return tps / 28 * gearRatio * 60;
    }
}
