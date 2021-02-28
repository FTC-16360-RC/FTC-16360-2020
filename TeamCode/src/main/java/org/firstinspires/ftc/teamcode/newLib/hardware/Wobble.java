package org.firstinspires.ftc.teamcode.newLib.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.newLib.Comms;

public class Wobble {

    enum Position {
        LOW,
        MEDIUM,
        HIGH;
    }

    HardwareMap hardwareMap;
    Servo arm1;
    Servo arm2;
    Servo hand;

    public Wobble() {
        hardwareMap = Comms.hardwareMap;
        arm1 = hardwareMap.get(Servo.class, "Arm 1");
        arm2 = hardwareMap.get(Servo.class, "Arm 2");
        hand = hardwareMap.get(Servo.class, "Hand");
    }

    public void setPosition(Position position) {
        switch (position) {
            case LOW:
                break;
            case MEDIUM:
                break;
            case HIGH:
                break;
        }
    }

}
