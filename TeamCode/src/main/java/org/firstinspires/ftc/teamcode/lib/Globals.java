package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Globals {

    public enum Teams {
        RED,
        BLU;
    }

    public static Teams team;
    public static double e = 0.000001;


    public double NormalizeAngle(double alpha) {
        alpha = alpha % 360;
        if (alpha < 0) {
            alpha = 360 + alpha;
        }
        return alpha;
    }

}
