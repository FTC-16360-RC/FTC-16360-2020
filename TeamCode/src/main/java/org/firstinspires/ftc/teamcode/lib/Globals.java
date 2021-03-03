package org.firstinspires.ftc.teamcode.lib;

import com.sun.tools.javac.code.TargetType;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.lib.datatypes.Vector;

public class Globals {

    public enum Alliance {
        RED,
        BLUE
    }

    public static Alliance alliance;

    public static Targets.TargetType targetType;

    public static Vector2D currentTarget;

    public static void updateTarget() {
        int targetYArrayIndex = 0;
        switch (targetType) {
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
        if(alliance==Alliance.RED)
            mirrorCoefficient = -1;
        currentTarget = new Vector2D(Targets.targetX, mirrorCoefficient*Targets.targetsY[targetYArrayIndex]);
    }

    public static double rpmToTicksPerSecond(double rpm, double gearRatio) {
        return rpm * 28 / gearRatio / 60;
    }

    public static double ticksPerSecondToRpm(double tps, double gearRatio) {
        return tps / 28 * gearRatio * 60;
    }
}
