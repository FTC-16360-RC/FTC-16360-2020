package org.firstinspires.ftc.teamcode.lib;

public class Globals {
    public static double rpmToTicksPerSecond(double rpm, double gearRatio) {
        return rpm * 28 / gearRatio / 60;
    }

    public static double ticksPerSecondToRpm(double tps, double gearRatio) {
        return tps / 28 * gearRatio * 60;
    }
}
