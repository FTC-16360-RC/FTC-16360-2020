package org.firstinspires.ftc.teamcode.lib;

import org.firstinspires.ftc.teamcode.lib.datatypes.Vector;

public class Control {

    double iErrorSum = 0;
    double iCount = 0;

    double lastError = 0;
    Vector K_VALUES;

    private Control(Vector K_VALUES) {
        this.K_VALUES = K_VALUES;
    }

    public double PID(double error) {
        double p_error;
        double i_error;
        double d_error;

        p_error = error;

        iErrorSum += error;
        iCount ++;
        i_error = iErrorSum / iCount;

        d_error = error - lastError;
        lastError = error;

        return ((p_error * K_VALUES.x) + (i_error * K_VALUES.y) + (d_error * K_VALUES.z) ) * K_VALUES.w;
    }
}
