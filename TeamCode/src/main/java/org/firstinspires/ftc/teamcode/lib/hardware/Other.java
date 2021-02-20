package org.firstinspires.ftc.teamcode.lib.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.Globals;

public class Other {

    Servo wobbleGoal;

    public Other() {
        wobbleGoal = Globals.hardwareMap.get(Servo.class, "wobbleGoal");
    }

}
