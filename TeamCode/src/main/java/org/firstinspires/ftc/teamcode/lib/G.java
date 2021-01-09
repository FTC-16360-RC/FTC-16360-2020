package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class G {

    public static double e = 0.000001;
    public HardwareMap hardwareMap;

    public enum a {
        NONE,
        SHOOTER,
        KEYBINDINGS,
        ALIGN_TO_POINT,
        INTAKE;
    }

    public enum i {
        NONE,
        SHOOT_ONE,
        SHOOT_THREE,
        NEXT_TARGET,
        PREVIOUS_TARGET,
        ENABLE_INTAKE,
        DISABLE_INTAKE,
        ENABLE_SHOOTER,
        DISABLE_SHOOTER,
        RESET_ORIENTATION,
        RECIEVE_POSITION,

        ENABLE_INTAKE_DEBUG,
        DISABLE_INTAKE_DEBUG,
        ENABLE_LIFT_DEBUG,
        DISABLE_LIFT_DEBUG,
        LOWER_INTAKE_DEBUG,
        ADJUST_FLAP_DEBUG,
        FEED_RING_DEBUG;
    }

    public double NormalizeAngle(double alpha) {
        alpha = alpha % 360;
        if (alpha < 0) {
            alpha = 360 + alpha;
        }
        return alpha;
    }

}
